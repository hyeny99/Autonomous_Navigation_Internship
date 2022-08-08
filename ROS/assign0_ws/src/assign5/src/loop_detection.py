#! /usr/bin/env python3
import scan_matching2 as icp

class Loop_closure():
    def __init__(self, first):
        self.first = first

    
    def detect_loop(self, second):
        _, _, _, is_converged = icp.icp(second, self.first)
        if is_converged:
            return True
        
        return False