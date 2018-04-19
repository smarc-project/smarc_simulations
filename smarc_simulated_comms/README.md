# Smarc_comms

Communication between agents and other entities. 

```
from smarc_msgs.msg import CommsMessage
```

CommsMessage:
```
Header header
string source_ns
string target_ns
string data
```

source_ns should be the agent's root namespace. Ex: 'lolo_auv_1/some/other/part' source_ns='lolo_auv_1'
same for target_ns. Alternatively, target_ns can be 'broadcast'. This will make it so that everyone in range receives the message.

data can be anything. If using python, piclikng is the easiest way to get a string representation of your data.

Publish CommsMessage to '/comms/inbound'.

Any messages to a an agent will be published to 'target_ns/comms_inbound'. Subscribe to that topic to receive the messages.
It is up to the agent to decide what to do with these messages. 

As of 18/04, only distance between agents is checked as a message receiving condition.

All communicators must have 'lolo' or 'sam' as the first part of their source_ns. Ex: 'lolo_asd_123', 'sam_the_greatest' are okay but 'my_name_is_sam' is not.
Acceptable names are collected from the list of all topics, so if there is no topic being published for an agent, it will not receive messages.


## TODO: 
Make distance check a rosparam.
Make rescan_period a rosparam.
Make it possible for arbitrarly named agents to connect.
Add other checks? Line of sight?

