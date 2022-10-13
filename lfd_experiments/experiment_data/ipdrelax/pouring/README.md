## Notes about the config.json

```
 "settings":
   {
        "calibration_settings": {
            "start_configuration": [0.5889033203125, 0.5130458984375, -0.585072265625, 0.61779296875, 0.98657421875, -0.8804697265625, -1.3759873046875],
            "calibration_pose":
            {
                "position": [1.0896, -0.04371, -0.15091],
                "orientation": [0.724142324757704, -0.011632070356660449, 0.687893404040879, 0.047804321047140386]    
            }
        },
       "recording_settings": {
           "start_configuration": [0.5889033203125, 0.5130458984375, -0.585072265625, 0.61779296875, 0.98657421875, -0.8804697265625, -1.3759873046875],
           "sampling_rate": 20,
           "constraint_trigger_mechanism": "web_trigger"
       },
       "labeling_settings": {
           "divisor": 20,
           "window": 10
       },
       "modeling_settings": {
           "planner": "RRTkConfigDefault",
           "culling_threshold": 12,
           "bandwidth": 0.15,
           "number_of_samples": 100
       },
       "raw_output_directory": "./raw_demonstrations/",
       "labeled_output_directory": "./labeled_demonstrations/",
       "tip_names": ["right_hand"]

   },
...
"robots": [
       {
           "class": "SawyerRobot",
           "init_args":
           {
               "item_id": 1,
               "reference_pose":
               {
                   "position": [0.578695253097807, -0.39773819035311436, -0.011090908106854147],
                   "orientation":[-0.7071318608127011, -0.02090705254793435, -0.011310713675474933, -0.7066820319871432]

               },
               "child_frame": "right_hand"
           }
       }

   ]
```

Since IPD relax uses the right_hand wrist as the FK link, we need to ensure this is what is being used to generate the robot's position for constraint evaluation and CCLfD modeling.
