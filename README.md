# threads_rospy_analysis

Different tests to get to know how many threads do different rospy implementations trigger.

Run `spawn_stuff_to_test.py` first. It will launch 20 topic (pub, sub), 20 AS (client, server) and 20 service (client, server).

Then open `rospy_threads_tests.py` and choose your test to know how many threads its using.

To know how many threads it's using you can run:

    ls -1 /proc/`ps axf | grep rospy_threads_tests.py | grep -v grep | awk '{print $1}'`/task/ | wc -l
    
Which counts the folders in `/proc/PID_OF_PROCESS/tasks`. Every folder is a thread.  

# The conclusion is:

## Topics:
Publisher: 1 thread per connection (0 not connected)

Subscriber: 1 thread per connection (0 not connected)
  
## Action servers:
Clients: 3 thread per connection (0 not connected)
  
Servers: 4 thread per connection (2 not connected)
  
## Services:
Clients: 0 threads
  
Servers: 0 threads
  

# As of CPU load
Some (not very meaningful) tests on my machine were performed with the conclusions:

## Topics: 
No extra CPU (just when actually doing something with them)
  
## Action servers:
Clients: No extra CPU
  
Servers: 2-3 % extra for every server
  
## Services: 
No extra CPU
