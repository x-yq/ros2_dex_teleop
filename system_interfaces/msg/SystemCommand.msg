# system_interfaces/msg/SystemCommand.msg  
# Message for system-wide commands

string command          # Command: get_status, urgent_stop, etc.
string sender           # Who sent the command (usually system_manager)
string[] parameters     # Optional command parameters
builtin_interfaces/Time timestamp