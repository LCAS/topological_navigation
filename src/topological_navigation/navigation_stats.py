#!/usr/bin/env python

from datetime import datetime

class nav_stats(object):
    
    def __init__(self, origin, target, topol_map):
        self.status= "active"
        self.origin=origin
        self.target=target
        self.topological_map = topol_map
        #self.date_at_node = datetime.now()
        self.set_start()
    
    def set_start(self):
        self.date_started = datetime.now()
        self.date_at_node = self.date_started

    def set_ended(self, node):
        self.final_node=node
        self.date_finished = datetime.now()
        self.get_operation_time()
        self.get_time_to_wp()
        
    def set_at_node(self):
        self.date_at_node = datetime.now()        
        
    def get_operation_time(self):
        operation_delta = self.date_finished - self.date_started
        self.operation_time = operation_delta.total_seconds()
        return self.operation_time

    def get_time_to_wp(self):
        if self.date_at_node != self.date_started :
            operation_delta = self.date_finished - self.date_at_node
            self.time_to_wp = operation_delta.total_seconds()
        else :
            self.time_to_wp = 0
        return self.time_to_wp
        
    def get_start_time_str(self):
        dt_text=self.date_started.strftime('%A, %B %d, at %H:%M:%S hours')
        return dt_text
        
    def get_finish_time_str(self):
        dt_text=self.date_finished.strftime('%A, %B %d, at %H:%M:%S hours')
        return dt_text