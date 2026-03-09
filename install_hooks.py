import os
import shutil
import stat
import sys

def install_hooks():
    hooks_dir = os.path.join(".git", "hooks")
    if not os.path.exists(hooks_dir):
        print("Error: .git/hooks directory not found. Are you in the root of the repository?")
        return

    hooks = {
        "post-checkout": "#!/bin/sh\n./.venv/Scripts/python.exe generate_version.py\n",
        "post-commit": "#!/bin/sh\n./.venv/Scripts/python.exe generate_version.py\n"
    }

    for hook_name, content in hooks.items():
        hook_path = os.path.join(hooks_dir, hook_name)
        
        print(f"Installing {hook_name}...")
        with open(hook_path, "w", newline='\n') as f:
            f.write(content)
        
        # Make the hook executable (important for Unix-like environments/Git Bash)
        st = os.stat(hook_path)
        os.chmod(hook_path, st.st_mode | stat.S_IEXEC)

    print("\nGit hooks installed successfully!")
    print("These hooks will automatically run 'generate_version.py' to update roborio/version.py after checkouts and commits.")

if __name__ == "__main__":
    install_hooks()
