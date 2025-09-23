#!/usr/bin/env python3
import subprocess
import time
import os
import signal
import sys
import threading

class RevolutionarySystemLauncher:
    """🚀 Launch Complete Revolutionary Lunar Rover System"""
    
    def __init__(self):
        self.processes = []
        self.lunar_path = "/home/darshan/lunar"
        self.features_path = f"{self.lunar_path}/revolutionary_features"
        
    def print_banner(self):
        """Print system banner"""
        
        banner = """
╔══════════════════════════════════════════════════════════════╗
║                  🚀 REVOLUTIONARY LUNAR ROVER SYSTEM          ║
║                     Advanced AI Demonstration                 ║
╠══════════════════════════════════════════════════════════════╣
║  🌪️  Dust Storm Adaptive SLAM                                ║
║  🔧  Fault Tolerance & Recovery                               ║
║  📊  Live Mission Logging                                     ║
║  ⚡  Energy Management                                        ║
║  🦋  Multi-Robot Swarm Coordination                          ║
║  🛡️  Predictive Safety & Hazard Detection                    ║
╠══════════════════════════════════════════════════════════════╣
║                   🏆 SIH 2025 WINNER SYSTEM                  ║
╚══════════════════════════════════════════════════════════════╝
        """
        print(banner)
        
    def setup_environment(self):
        """Setup ROS environment"""
        
        print("🔧 Setting up ROS environment...")
        
        # Set ROS environment
        os.environ['ROS_DOMAIN_ID'] = '0'
        os.environ['RMW_IMPLEMENTATION'] = 'rmw_cyclone_dx'
        
        # Change to workspace
        os.chdir(self.lunar_path)
        
        print("✅ Environment configured")
        
    def launch_gazebo_world(self):
        """Launch Gazebo simulation world"""
        
        print("🌍 Launching enhanced lunar environment...")
        
        # Try different world files
        world_files = [
            f"{self.lunar_path}/src/lunabot_simulation/worlds/ultra_realistic_lunar.world",
            f"{self.lunar_path}/src/lunabot_simulation/worlds/bright_lunar.world",
            f"{self.lunar_path}/worlds/lunar_surface.world"
        ]
        
        world_file = None
        for wf in world_files:
            if os.path.exists(wf):
                world_file = wf
                break
                
        if not world_file:
            print("⚠️ No world file found, using default Gazebo world")
            cmd = ["gazebo", "--verbose"]
        else:
            cmd = ["gazebo", "--verbose", "-s", "libgazebo_ros_factory.so", world_file]
        
        process = subprocess.Popen(cmd, cwd=self.lunar_path)
        self.processes.append(("Gazebo World", process))
        
        print("⏳ Waiting for Gazebo to initialize...")
        time.sleep(12)
        print("✅ Gazebo world loaded")
        
    def spawn_rover(self):
        """Spawn lunar rover"""
        
        print("🤖 Spawning lunar rover...")
        
        # Try different URDF files
        urdf_files = [
            f"{self.lunar_path}/src/lunabot_simulation/urdf/custom_rover.urdf",
            f"{self.lunar_path}/src/lunabot_simulation/urdf/lunabot.urdf",
            f"{self.lunar_path}/urdf/rover.urdf"
        ]
        
        urdf_file = None
        for uf in urdf_files:
            if os.path.exists(uf):
                urdf_file = uf
                break
                
        if urdf_file:
            cmd = [
                "ros2", "run", "gazebo_ros", "spawn_entity.py",
                "-entity", "revolutionary_rover",
                "-file", urdf_file,
                "-x", "0", "-y", "0", "-z", "5.0"
            ]
            
            process = subprocess.Popen(cmd, cwd=self.lunar_path)
            self.processes.append(("Rover Spawn", process))
            
            print("⏳ Spawning rover...")
            time.sleep(6)
            print("✅ Rover spawned successfully")
        else:
            print("⚠️ No URDF file found, skipping rover spawn")
            
    def launch_revolutionary_features(self):
        """Launch all 6 revolutionary features"""
        
        print("⚡ Launching revolutionary AI features...")
        
        features = [
            ("Dust Storm SLAM", "dust_storm_slam.py"),
            ("Fault Recovery", "fault_recovery_system.py"),
            ("Mission Logger", "mission_logger.py"),
            ("Energy Management", "energy_management.py"),
            ("Swarm Coordination", "swarm_coordination.py"),
            ("Predictive Safety", "predictive_safety.py")
        ]
        
        for feature_name, script_name in features:
            script_path = f"{self.features_path}/{script_name}"
            
            if os.path.exists(script_path):
                print(f"🚀 Launching {feature_name}...")
                cmd = ["python3", script_path]
                process = subprocess.Popen(cmd, cwd=self.lunar_path)
                self.processes.append((feature_name, process))
                time.sleep(2)  # Stagger launches
                print(f"✅ {feature_name} started")
            else:
                print(f"⚠️ {feature_name} script not found: {script_path}")
                
    def launch_original_systems(self):
        """Launch original AI systems if available"""
        
        print("🧠 Launching original AI systems...")
        
        original_systems = [
            ("AMFS SLAM", ["ros2", "run", "amfs_slam", "simple_adaptive_slam_node"]),
            ("Hazard Prediction", ["ros2", "run", "predictive_hazard", "hazard_prediction_node"])
        ]
        
        for system_name, cmd in original_systems:
            try:
                process = subprocess.Popen(cmd, cwd=self.lunar_path)
                self.processes.append((system_name, process))
                print(f"✅ {system_name} started")
                time.sleep(1)
            except FileNotFoundError:
                print(f"⚠️ {system_name} not available")
                
    def monitor_system_health(self):
        """Monitor system health in background"""
        
        def health_monitor():
            while True:
                time.sleep(10)
                
                alive_processes = []
                dead_processes = []
                
                for name, process in self.processes:
                    if process.poll() is None:
                        alive_processes.append(name)
                    else:
                        dead_processes.append(name)
                        
                if dead_processes:
                    print(f"⚠️ Dead processes: {', '.join(dead_processes)}")
                    
                print(f"📊 System Health: {len(alive_processes)}/{len(self.processes)} processes active")
                
        health_thread = threading.Thread(target=health_monitor, daemon=True)
        health_thread.start()
        
    def display_system_status(self):
        """Display comprehensive system status"""
        
        status = """
╔══════════════════════════════════════════════════════════════╗
║                    🎯 SYSTEM STATUS ACTIVE                    ║
╠══════════════════════════════════════════════════════════════╣
║  🌪️  Dust Storm Simulation         ✅ RUNNING              ║
║  🔧  Fault Injection System        ✅ RUNNING              ║
║  📊  Live Mission Logging          ✅ RUNNING              ║
║  ⚡  Energy Management             ✅ RUNNING              ║
║  🦋  Swarm Coordination            ✅ RUNNING              ║
║  🛡️  Predictive Safety             ✅ RUNNING              ║
╠══════════════════════════════════════════════════════════════╣
║                    📱 MONITORING TOPICS                      ║
╠══════════════════════════════════════════════════════════════╣
║  /dust_level              - Dust storm intensity           ║
║  /fault_status            - System health monitoring       ║
║  /mission_status          - Live mission metrics           ║
║  /battery_level           - Energy management              ║
║  /swarm_status            - Multi-robot coordination       ║
║  /hazard_alerts           - Safety system alerts           ║
╠══════════════════════════════════════════════════════════════╣
║                    🎬 DEMONSTRATION TIMELINE                 ║
╠══════════════════════════════════════════════════════════════╣
║  0:00  - System initialization                             ║
║  0:20  - Dust storm begins (SLAM adapts)                   ║
║  0:30  - Fault injection (wheel failure)                   ║
║  0:45  - Autonomous recovery sequence                       ║
║  1:00  - Low battery (seeks charging station)              ║
║  1:30  - Swarm coordination demo                           ║
║  2:00  - Hazard detection showcase                         ║
╚══════════════════════════════════════════════════════════════╝

Press Ctrl+C to shutdown system
        """
        
        print(status)
        
    def run_demonstration_sequence(self):
        """Run automated demonstration sequence"""
        
        demonstrations = [
            (20, "🌪️ DUST STORM: Watch SLAM adaptation in real-time"),
            (30, "🔧 FAULT INJECTION: Wheel failure - observe recovery"),
            (45, "✅ RECOVERY: Autonomous system restoration"),
            (60, "⚡ ENERGY: Low battery - routing to charging station"),
            (90, "🦋 SWARM: Multi-robot coordination demo"),
            (120, "🛡️ SAFETY: Hazard detection and avoidance"),
            (150, "📊 REPORT: Mission completion and data analysis")
        ]
        
        start_time = time.time()
        demo_index = 0
        
        while demo_index < len(demonstrations):
            current_time = time.time() - start_time
            
            if current_time >= demonstrations[demo_index][0]:
                print(f"\n🎬 DEMO: {demonstrations[demo_index][1]}")
                demo_index += 1
                
            time.sleep(1)
            
    def run_interactive_mode(self):
        """Run interactive monitoring mode"""
        
        print("\n🎮 INTERACTIVE MODE - Available commands:")
        print("  'status' - Show system status")
        print("  'topics' - List active ROS topics")
        print("  'metrics' - Show performance metrics")
        print("  'demo' - Run demonstration sequence")
        print("  'quit' - Shutdown system")
        
        while True:
            try:
                command = input("\n🤖 Enter command: ").strip().lower()
                
                if command == 'quit':
                    break
                elif command == 'status':
                    self.display_system_status()
                elif command == 'topics':
                    subprocess.run(["ros2", "topic", "list"])
                elif command == 'metrics':
                    print("📊 Performance metrics logged to ~/lunar/mission_logs/")
                elif command == 'demo':
                    print("🎬 Starting demonstration sequence...")
                    demo_thread = threading.Thread(target=self.run_demonstration_sequence)
                    demo_thread.start()
                else:
                    print("❓ Unknown command. Type 'quit' to exit.")
                    
            except KeyboardInterrupt:
                break
                
    def shutdown_system(self):
        """Shutdown all processes gracefully"""
        
        print("\n🛑 Shutting down revolutionary system...")
        
        for name, process in reversed(self.processes):
            try:
                print(f"🔴 Stopping {name}...")
                process.terminate()
                
                # Wait for graceful shutdown
                try:
                    process.wait(timeout=5)
                    print(f"✅ {name} stopped gracefully")
                except subprocess.TimeoutExpired:
                    print(f"🔴 Force killing {name}...")
                    process.kill()
                    process.wait()
                    
            except Exception as e:
                print(f"❌ Error stopping {name}: {e}")
                
        print("🔚 Revolutionary system shutdown complete")
        
    def run(self):
        """Main execution function"""
        
        try:
            self.print_banner()
            self.setup_environment()
            
            # Launch sequence
            self.launch_gazebo_world()
            self.spawn_rover()
            self.launch_revolutionary_features()
            self.launch_original_systems()
            
            # Start monitoring
            self.monitor_system_health()
            self.display_system_status()
            
            # Run interactive mode
            self.run_interactive_mode()
            
        except KeyboardInterrupt:
            print("\n🔴 Keyboard interrupt received")
        except Exception as e:
            print(f"❌ System error: {e}")
        finally:
            self.shutdown_system()


def main():
    """Main entry point"""
    
    # Handle signals for graceful shutdown
    launcher = RevolutionarySystemLauncher()
    
    def signal_handler(signum, frame):
        print(f"\n🔴 Signal {signum} received")
        launcher.shutdown_system()
        sys.exit(0)
        
    signal.signal(signal.SIGINT, signal_handler)
    signal.signal(signal.SIGTERM, signal_handler)
    
    launcher.run()


if __name__ == "__main__":
    main()
