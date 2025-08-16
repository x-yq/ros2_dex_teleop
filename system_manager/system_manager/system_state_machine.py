import json
import yaml
import os
from datetime import datetime

class SystemStateMachine:
    """Manages system operational states"""
    
    STATES = {
        'DISCONNECTED': 'disconnected',
        'RUNNING': 'running',
        'PAUSED': 'paused'
    }

    ## TODO: not only transition, but also some real function to control the system

    
    def __init__(self):
        self.current_state = self.STATES['DISCONNECTED']
        self.previous_state = None
        self.state_history = []
        
    def run(self):
        """Start system operation"""
        if self.current_state == self.STATES['PAUSED']:
            self._transition_to(self.STATES['RUNNING'])
            return True
        return False
        
    def pause(self):
        """Pause system operation"""
        if self.current_state == self.STATES['RUNNING']:
            self._transition_to(self.STATES['PAUSED'])
            return True
        return False
        
    def urgent_stop(self):
        """Emergency stop"""
        self._transition_to(self.STATES['PAUSED'])
        return True
        
    def connect(self):
        """Transition to connected state"""
        if self.current_state == self.STATES['DISCONNECTED']:
            self._transition_to(self.STATES['PAUSED'])
            return True
        return False
        
    def error(self):
        """Transition to error state"""
        self._transition_to(self.STATES['DISCONNECTED'])
        return True
        
    def get_current_system_state(self):
        """Get current system state"""
        return self.current_state
        
    def update_current_system_state(self, new_state):
        """Update system state (with validation)"""
        if new_state in self.STATES.values():
            self._transition_to(new_state)
            return True
        return False
        
    def _transition_to(self, new_state):
        """Internal method to handle state transitions"""
        self.previous_state = self.current_state
        self.current_state = new_state
        self.state_history.append({
            'from': self.previous_state,
            'to': self.current_state,
            'timestamp': datetime.now()
        })
        
    def get_state_history(self):
        """Get state transition history"""
        return self.state_history
