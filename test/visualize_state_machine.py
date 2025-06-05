#!/usr/bin/env python3
"""
State Machine Visualization Tool
Creates a visual representation of the orchestration state machine
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import json
from datetime import datetime
import os

class StateMachineVisualizer(Node):
    def __init__(self):
        super().__init__('state_visualizer')
        
        # Subscribe to state changes
        self.state_change_sub = self.create_subscription(
            String, '/state_changes', self.handle_state_change, 10)
        
        # Track state transitions
        self.transitions = []
        self.current_tasks = {}
        
        self.get_logger().info('State Machine Visualizer started')
        
        # Create output directory
        self.output_dir = os.path.expanduser("~/.ros/state_visualizations")
        os.makedirs(self.output_dir, exist_ok=True)
    
    def handle_state_change(self, msg):
        """Handle and visualize state changes."""
        try:
            state_data = json.loads(msg.data)
            self.transitions.append(state_data)
            
            # Extract task information
            reason = state_data.get('reason', '')
            task_id = None
            if 'Task' in reason:
                try:
                    task_id = reason.split('Task ')[1].split(' ')[0]
                except:
                    pass
            
            # Update current tasks
            if task_id and state_data.get('entity') == 'task':
                if task_id not in self.current_tasks:
                    self.current_tasks[task_id] = {
                        'states': [],
                        'start_time': state_data.get('timestamp')
                    }
                
                self.current_tasks[task_id]['states'].append({
                    'from': state_data.get('from_state'),
                    'to': state_data.get('to_state'),
                    'timestamp': state_data.get('timestamp'),
                    'reason': reason
                })
            
            # Print visual representation
            self.print_state_transition(state_data, task_id)
            
            # Save visualization
            self.save_visualization()
            
        except json.JSONDecodeError:
            self.get_logger().error(f"Invalid JSON in state change: {msg.data}")
        except Exception as e:
            self.get_logger().error(f"Error processing state change: {e}")
    
    def print_state_transition(self, state_data, task_id):
        """Print a visual representation of the state transition."""
        entity = state_data.get('entity', 'unknown')
        from_state = state_data.get('from_state', 'unknown')
        to_state = state_data.get('to_state', 'unknown')
        timestamp = state_data.get('timestamp', '')
        reason = state_data.get('reason', '')
        
        # Create visual arrow
        arrow = self.get_state_arrow(from_state, to_state)
        
        # Color coding for different states
        color = self.get_state_color(to_state)
        
        print(f"\n{timestamp[:19]}")
        if task_id:
            print(f"📋 Task {task_id}: {from_state} {arrow} {to_state}")
        else:
            print(f"🔄 {entity}: {from_state} {arrow} {to_state}")
        
        if reason:
            print(f"   💬 {reason}")
        
        # Show current system state
        self.print_system_overview()
    
    def get_state_arrow(self, from_state, to_state):
        """Get appropriate arrow for state transition."""
        arrows = {
            ('CREATED', 'PENDING'): '➡️',
            ('PENDING', 'ACTIVE'): '🔍',
            ('ACTIVE', 'PICKING'): '🤖',
            ('PICKING', 'WAITING_FOR_DELIVERY'): '✅',
            ('WAITING_FOR_DELIVERY', 'DELIVERING'): '🚚',
            ('DELIVERING', 'COMPLETE'): '🎯',
            ('PICKING', 'FAILED'): '❌',
            ('ACTIVE', 'FAILED'): '❌',
            ('DELIVERING', 'FAILED'): '❌'
        }
        return arrows.get((from_state, to_state), '→')
    
    def get_state_color(self, state):
        """Get color coding for states."""
        colors = {
            'PENDING': '🟡',
            'ACTIVE': '🔵',
            'PICKING': '🟠',
            'WAITING_FOR_DELIVERY': '🟢',
            'DELIVERING': '🟣',
            'COMPLETE': '✅',
            'FAILED': '❌'
        }
        return colors.get(state, '⚪')
    
    def print_system_overview(self):
        """Print current system overview."""
        if not self.current_tasks:
            return
        
        print("   📊 Current Tasks:")
        for task_id, task_info in self.current_tasks.items():
            if task_info['states']:
                latest_state = task_info['states'][-1]['to']
                color = self.get_state_color(latest_state)
                print(f"      Task {task_id}: {color} {latest_state}")
    
    def save_visualization(self):
        """Save current state visualization to file."""
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        filename = os.path.join(self.output_dir, f"state_transitions_{timestamp}.json")
        
        visualization_data = {
            'timestamp': datetime.now().isoformat(),
            'transitions': self.transitions,
            'current_tasks': self.current_tasks,
            'summary': self.generate_summary()
        }
        
        try:
            with open(filename, 'w') as f:
                json.dump(visualization_data, f, indent=2)
        except Exception as e:
            self.get_logger().error(f"Failed to save visualization: {e}")
    
    def generate_summary(self):
        """Generate summary statistics."""
        total_transitions = len(self.transitions)
        total_tasks = len(self.current_tasks)
        
        # Count states
        state_counts = {}
        for transition in self.transitions:
            to_state = transition.get('to_state')
            state_counts[to_state] = state_counts.get(to_state, 0) + 1
        
        # Count completed vs failed tasks
        completed_tasks = 0
        failed_tasks = 0
        active_tasks = 0
        
        for task_info in self.current_tasks.values():
            if task_info['states']:
                latest_state = task_info['states'][-1]['to']
                if latest_state == 'COMPLETE':
                    completed_tasks += 1
                elif latest_state == 'FAILED':
                    failed_tasks += 1
                else:
                    active_tasks += 1
        
        return {
            'total_transitions': total_transitions,
            'total_tasks': total_tasks,
            'completed_tasks': completed_tasks,
            'failed_tasks': failed_tasks,
            'active_tasks': active_tasks,
            'state_counts': state_counts
        }
    
    def print_final_summary(self):
        """Print final summary when shutting down."""
        summary = self.generate_summary()
        
        print("\n" + "="*60)
        print("STATE MACHINE VISUALIZATION SUMMARY")
        print("="*60)
        print(f"Total Transitions: {summary['total_transitions']}")
        print(f"Total Tasks: {summary['total_tasks']}")
        print(f"Completed Tasks: {summary['completed_tasks']}")
        print(f"Failed Tasks: {summary['failed_tasks']}")
        print(f"Active Tasks: {summary['active_tasks']}")
        
        print("\nState Distribution:")
        for state, count in summary['state_counts'].items():
            color = self.get_state_color(state)
            print(f"  {color} {state}: {count}")
        
        print(f"\nVisualization saved to: {self.output_dir}")
        print("="*60)

def main(args=None):
    rclpy.init(args=args)
    node = StateMachineVisualizer()
    
    try:
        print("\n🎨 State Machine Visualizer Started")
        print("Monitoring state transitions on /state_changes...")
        print("Press Ctrl+C to stop and see summary\n")
        
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.print_final_summary()
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
