import os
file = os.path.join(os.path.abspath(os.path.dirname(__file__)), "continue.txt")

def set_continue(state:bool=True):
    with open(file, 'w') as f: f.write(str(state))

def is_continue():
    with open(file, 'r') as f: return f.read() == "True"