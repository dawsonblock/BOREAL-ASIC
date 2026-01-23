
import sys
import os
import subprocess

def run():
    log_file = "training_log_verify.txt"
    print(f"Starting training run, logging to {log_file}...")
    
    with open(log_file, "w") as f:
        # Run the demo
        cmd = [sys.executable, "demos/panda_bandit_demo.py", "--xml", "panda_table_cube.xml", "--episodes", "50", "--render-every", "50"]
        f.write(f"Running command: {' '.join(cmd)}\n")
        f.flush()
        
        env = os.environ.copy()
        env["PYTHONPATH"] = os.getcwd()
        
        try:
            result = subprocess.run(cmd, stdout=f, stderr=subprocess.STDOUT, env=env, cwd=os.getcwd())
            if result.returncode == 0:
                f.write("\nSUCCESS: Training completed.\n")
                print("Training finished successfully.")
            else:
                f.write(f"\nFAILED with code {result.returncode}\n")
                print(f"Training failed with code {result.returncode}.")
        except Exception as e:
            f.write(f"\nEXCEPTION: {e}\n")
            print(f"Execution failed: {e}")

if __name__ == "__main__":
    run()
