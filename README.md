Topological Navigation
======================

Utility package containing a node that can be used to manage twitter accounts from ROS


## Installation

Install Twython via pip

```
    $ pip install twython
```

or, with easy_install

```
    $ easy_install twython
```

## Starting Out


  * Go to ` https://dev.twitter.com/apps ` and register an application
  * Go to the settings tab and chage permitions to ` Read, Write and Access direct messages `
  * Go back to the Details tab and at the botton hit the ` Create Access Token Button `
  * Go to OAuth tool tab and get the <strong>Consumer key</strong>, <strong>Consumer secret</strong>, <strong>Access token</strong> and <strong>Access token secret</strong> and save them on `/opt/strands/strands_catkin_ws/src/strands_utils/strands_datacentre/defaults/twitter_params.yaml`
  * Launch the strands_datacentre: 
  ``` roslaunch strands_datacentre datacentre.launch```
  * Save the parameters on your locals collection:

  ```rosservice call /config_manager/save_param /twitter/appKey```

  ```rosservice call /config_manager/save_param /twitter/appSecret```

  ```rosservice call /config_manager/save_param /twitter/oauthToken```

  ```rosservice call /config_manager/save_param /twitter/oauthTokenSecret```
  * Now you are ready to go!

## Tweeting

Run the Strands_tweets node
```rosrun strands_tweets tweet.py```

You can send a tweet by calling the `/strands_tweets/Tweet` service like this:

```rosservice call /strands_tweets/Tweet 'Whatever you want to say' false```

You can also send a tweet using the `tweet_test` client like this:

```rosrun strands_tweets tweet_test.py 'Whatever you want to say'```
