#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
from topological_navigation_msgs.msg import *
from topological_navigation_msgs.srv import *
from mongodb_store.message_store import MessageStoreProxy
import actionlib
import pytz
from datetime import datetime, timedelta, time, date
import pymongo
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.backends.backend_pdf import PdfPages
from math import fabs

# import mongodb_store_msgs.srv as dc_srv
# import mongodb_store.util as dc_util
# from mongodb_store.message_store import MessageStoreProxy
# from datetime import datetime, timedelta, time, date

# import matplotlib.patches as mpatches
# import numpy as np
# import matplotlib.pyplot as plt

# import argparse
# import cmd


def model_client():
    #Creating monitored navigation client    
    rospy.loginfo("Getting model build server")
    client = actionlib.SimpleActionClient('topological_prediction/build_temporal_model', BuildTopPredictionAction)
    client.wait_for_server()
    rospy.loginfo(" ...done")
    return client

def prediction_client():
    rospy.wait_for_service('topological_prediction/predict_edges')
    return rospy.ServiceProxy('topological_prediction/predict_edges', PredictEdgeState)


def to_epoch(ptime):
    return int((ptime - datetime(1970,1,1, tzinfo=ptime.tzinfo)).total_seconds())

def python_to_rostime(ptime):
    return rospy.Time((ptime - datetime(1970,1,1, tzinfo=ptime.tzinfo)).total_seconds())

def evaluate_prediction(predictor, epoch, stats):
    try:
        response = predictor(rospy.Time(epoch))
        
        errors = []

        for nav_stat in stats:
            # ignore the stay in one place transitions
            if nav_stat.edge_id != 'none':
                predicted_duration = response.durations[response.edge_ids.index(nav_stat.edge_id)]
                actual_duration = nav_stat.operation_time

                predicted_success = response.probs[response.edge_ids.index(nav_stat.edge_id)]
                actual_success = 1 if nav_stat.status == 'success' else 0

                errors.append((nav_stat.edge_id, actual_duration - predicted_duration.to_sec(), fabs(actual_success - predicted_success)))

        return errors

    except rospy.ServiceException as e:
        print "Service call failed: %s"%e


def write_stats(window_start, window_end, nav_stats):

    if len(nav_stats) == 0:
        return

    filename = '{2}__{0}__{1}.csv'.format(window_start, window_end, nav_stats[-1][0].topological_map)
    print 'Writing data for %s to %s, total stats %s, to %s' % (window_start, window_end, len(nav_stats), filename)

    # If these have time of start in the first column, success/fail in the
    # second column and time it took to traverse in the third one, then I
    # could try to create some nice stats out of it.

    with open(filename, 'w') as text_file:
        for stat, meta in nav_stats:            
            if stat.edge_id != 'none':
                text_file.write('{0}, {1}, {2}, {3}\n'.format(stat.edge_id, meta['epoch'], stat.status, stat.operation_time))
 

def export_data(tmap, data_window_start, data_window_end, data_window_size, data_windows):


    msg_store = MessageStoreProxy(collection='nav_stats')

    for window in range(data_windows):

        window_start = data_window_start + data_window_size * window
        window_end = data_window_end + data_window_size * window

        # get nav stats for window
        meta_query = {"epoch" : {"$gt": to_epoch(window_start), "$lt" : to_epoch(window_end)}}
        nav_stats = msg_store.query(NavStatistics._type, {'topological_map': tmap}, meta_query)
 
        write_stats(window_start, window_end, nav_stats)
  


def collect_predictions(predictor_name, now, tmap, training_window_start, training_window_end, training_window_size, training_windows, testing_window_start, testing_window_end):

    builder = model_client()
    predictor = prediction_client()

    msg_store = MessageStoreProxy(collection='nav_stats')

    host = rospy.get_param('mongodb_host', 'localhost')
    port = rospy.get_param('mongodb_port', '62345')
    client = pymongo.MongoClient(host,port)    

    results_collection = client['nav_results'][tmap]



    meta_query = {"epoch" : {"$gt": to_epoch(testing_window_start), "$lt" : to_epoch(testing_window_end)}}  
    test_stats = msg_store.query(NavStatistics._type, {'topological_map': tmap}, meta_query)

    test_epochs = {}

    test_stat_count = 0
    # group nav stats by epoch for faster processing later
    for nav_stat, nav_stat_meta in test_stats:
        if nav_stat_meta['epoch'] not in test_epochs:
            # print nav_stat_meta['epoch']
            test_epochs[nav_stat_meta['epoch']] = []

        test_epochs[nav_stat_meta['epoch']].append(nav_stat)        
        test_stat_count += 1


    print 'Testing with %s stats at %s different epochs' % (test_stat_count, len(test_epochs)) 
    

    processed = 0
    increment = 100
    increment_count = 100

    for window in range(training_windows):

        window_start = training_window_start
        window_end = training_window_end + training_window_size * window

        # get nav stats for window
        meta_query = {"epoch" : {"$gt": to_epoch(window_start), "$lt" : to_epoch(window_end)}}
        nav_stats = msg_store.query(NavStatistics._type, {'topological_map': tmap}, meta_query)
        print 'Building model for window %s to %s, total stats %s' % (training_window_start, training_window_end + training_window_size * window, len(nav_stats))

        build_goal = BuildTopPredictionGoal(python_to_rostime(window_start), python_to_rostime(window_end))
        builder.send_goal(build_goal)
        builder.wait_for_result()

        # now test each nav stat from the windows
        result_set = '%s___%s' % (window_start, window_end)
        for epoch, stats in test_epochs.items():
            results = evaluate_prediction(predictor, epoch, stats)
            result_doc = {'result_time': now, 'training_window_start': window_start, 'training_window_end': window_end, 'predictor' : predictor_name, 'epoch' : epoch, 'results': results} 
            results_collection.insert(result_doc)
            processed += len(results)

            if processed > increment_count:
                print 'Processed %s/%s' % (processed, test_stat_count)
                increment_count += increment

    print 'Processed %s/%s' % (processed, test_stat_count)

    return now




edge_id_to_x = {}

def visualise_window(result_time, predictor, results_collection, window_start, window_end, colour = 'b', label = None, duration_ax = None, success_ax = None):
    # get all results for this set
    # and build up answers by edge
    edge_results = {}    
    edge_prob_results = {}

    query =  {'result_time': result_time, 'predictor': predictor, 'training_window_start': window_start, 'training_window_end': window_end}

    print window_start

    for result in results_collection.find(query):
        # duration 
        for edge_id, error, prob_error in result['results']:
            if edge_id not in edge_results:
                edge_results[edge_id] = []                    
                edge_prob_results[edge_id] = []    

            if edge_id not in edge_id_to_x:
                edge_id_to_x[edge_id] = (len(edge_id_to_x) + 1) * 10

            edge_results[edge_id].append(error)            
            edge_prob_results[edge_id].append(prob_error)

    # now we have a list of errors per edge
    edge_ids = []
    real_edge_ids = []
    edge_means = []
    edge_stddevs = []


    for edge_id, edge_errors in edge_results.items():
        edge_errors = np.absolute(np.array(edge_errors))
        edge_ids.append(edge_id_to_x[edge_id])
        real_edge_ids.append(edge_id)
        edge_means.append(edge_errors.mean())
        edge_stddevs.append(edge_errors.std())

    edge_prob_means = []
    edge_prob_stddevs = []

        # work out mean prediction
    for edge_id in real_edge_ids:
        edge_errors = np.array(edge_prob_results[edge_id])
        edge_prob_means.append(edge_errors.mean())
        edge_prob_stddevs.append(edge_errors.std())

    # show edges and error
    duration_ax.errorbar(edge_ids, edge_means, edge_stddevs, linestyle='None', marker='^', color = colour, label = label)
    success_ax.errorbar(edge_ids, edge_prob_means, edge_prob_stddevs, linestyle='None', marker='^', color = colour, label = label)


def visualise_results(tmap, result_time):
    # first reconstruct windows from the data

    host = rospy.get_param('mongodb_host', 'localhost')
    port = rospy.get_param('mongodb_port', '62345')
    client = pymongo.MongoClient(host,port)    
    
    results_collection = client['nav_results'][tmap]

    predictors = results_collection.find({'result_time': result_time}, {'predictor': 1})
    predictors = set([predictor['predictor'] for predictor in  predictors])


    # gather in the unique windows that defined the training regime

    predictor_windows = []
    for predictor in predictors:        
        print predictor
        window_dates = {}
        for window in results_collection.find({'result_time': result_time, 'predictor': predictor}, {'training_window_start': 1, 'training_window_end':1}): 
            if window['training_window_start'] not in window_dates:
                window_dates[window['training_window_start']] = []
            window_dates[window['training_window_start']].append(window['training_window_end'])

        windows = []
        for start, ends in window_dates.items():
            ends = set(ends)        
            windows += [[predictor, start, end] for end in ends] 


        predictor_windows += windows


    predictor_windows.sort(key=lambda w: w[2])
    predictor_windows.sort(key=lambda w: w[0])
    print predictor_windows


    colour_map = plt.get_cmap('Paired')
    colours = [colour_map(i * 30) for i in range(len(predictor_windows))]

    count = 0

    with PdfPages('{0}_nav_stats.pdf'.format(tmap)) as pdf:
    
        plt.figure(figsize=(40,5))
        duration_ax = plt.subplot(2, 1, 1) 
        success_ax = plt.subplot(2, 1, 2) 

        for window in predictor_windows:
            print window
            visualise_window(result_time, window[0], results_collection, window[1], window[2], colours[count], window[0] + str(count), duration_ax, success_ax)
            count += 1

            for edge, x in edge_id_to_x.items():
                edge_id_to_x[edge] = x + 1

        plt.legend()
        # plt.show()
        pdf.savefig()  # saves the current figure into a pdf page
        plt.close()


def do_g4s_predictions(version=1):
    # get all nav stats
    tz = pytz.timezone(pytz.country_timezones['gb'][0])

    training_window_start = datetime(2015,5,6,8,0,tzinfo=tz)
    training_window_end = datetime(2015,5,12,18,0,tzinfo=tz)
    training_window_size = timedelta(days=7)
    training_windows = 4

    testing_window_start = datetime(2015,6,4,8,0,tzinfo=tz)
    testing_window_end = datetime(2015,6,10,18,0,tzinfo=tz)

    tmap = 'g4s_tmap'
    collect_predictions('mean', version, tmap, training_window_start, training_window_end, training_window_size, training_windows, testing_window_start, testing_window_end)


def do_aaf_predictions():

    tz = pytz.timezone(pytz.country_timezones['at'][0])
    training_window_start = datetime(2015,5,18,8,0,tzinfo=tz)
    training_window_end = datetime(2015,5,24,21,0,tzinfo=tz)
    training_window_size = timedelta(days=7)
    training_windows = 3

    testing_window_start = training_window_start + training_windows * training_window_size
    testing_window_end = training_window_end + training_windows * training_window_size

    tmap = 'aaf_deployment'

    collect_predictions('fremen', 1, tmap, training_window_start, training_window_end, training_window_size, training_windows, testing_window_start, testing_window_end)



def do_g4s_data_export():
    # g4s data export
    tz = pytz.timezone(pytz.country_timezones['gb'][0])
    data_window_start = datetime(2015,5,6,8,0,tzinfo=tz)
    data_window_end = datetime(2015,5,12,18,0,tzinfo=tz)
    data_window_size = timedelta(days=7)
    data_windows = 5
    tmap = 'g4s_tmap'
    export_data(tmap, data_window_start, data_window_end, data_window_size, data_windows)


def do_aaf_data_export():
    tz = pytz.timezone(pytz.country_timezones['at'][0])
    data_window_start = datetime(2015,5,18,8,0,tzinfo=tz)
    data_window_end = datetime(2015,5,24,21,0,tzinfo=tz)
    data_window_size = timedelta(days=7)
    data_windows = 5
    tmap = 'aaf_deployment'

    export_data(tmap, data_window_start, data_window_end, data_window_size, data_windows)



if __name__ == '__main__':
    rospy.init_node("topological_prediction_anlysis")


    # get all nav stats
    version=2
    do_g4s_predictions(version)
    visualise_results('g4s_tmap', version)



        
