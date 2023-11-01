# AgRobot version 2

DSLAB AgRobot project from 2022-2024. 

Detailed introduction in [here](https://dslab-agrobot.github.io/AgRobot2/)


The file structure should be:

```
.
├── LICENSE
├── README.md
└── src
    ├── management_scripts
    ├── software
    └── tools
```
Here the defination of `src` folders: 

- `software` contains resources for ROS, modbus, AI. 
- `tools` for misc scripts, like file transfer, image crop for AI, blablabla. 
- `management_scripts`, Enviroment management for this robot and corresponding package. e.g., flash the system of the NVDIA-NX, python and ROS setup.
