Check out the config_commented.json for some notes about what goes into the configuration files.

Recording a new skill:

1) Run ar4lfd.launch file.
2) Run modeling_keyboard_commands.py node.
3) Run recording_keyboard_commands.py node.
4) Run the lfd script of your choice (ar4lfd.py)

The lfd scripts have two modes. The top-level modeling mode where one can train a model, 
serialize models, etc,. By typing 'record' in the modeling keyboard controller console, 
you can enter a recording mode. In this recording mode, you can record, capture, discard 
demonstrations.

The recording console will output the following:

```
Keyboard interface for collecting demonstrations:
              'r' - Record
              'q' - Quit Recording Session
              'd' - Discard current demo while recording.
              'c' - Capture current demo while recording.
              's' - Move to start configuration
```

Generally you can perform the following procedure:

For each demonstration session:
s -> move to start
r -> Record a demonstration, this will immediately start recording and put the robot 
     into zero-g mode so be ready.
     
Either
c -> Captures the current demonstration and ends recording the demonstrations.
or
d -> Discards the current demonstration and ends recording.

Finally,

q -> Quits the recording, and returns the captured demonstration trajectories (if 
     there are any) to the modeling / study controller.


Once you have returned back to the modeling mode, you should type 'train' to learn your 
keyframe distributions. You should also type 'save' to output your raw and labalened 
demonstrations to the designated output directory.
