# system_interfaces/msg/ComponentStatus.msg
# Message for component status updates

string layer_cn_id      # Component node ID
string status           # Component status: disconnected, ready, running, paused, stopped, error
builtin_interfaces/Time timestamp