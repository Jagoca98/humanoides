# Humanoid Robots :robot:

## Webots :lady_beetle:

This project contains the files needed to move Teo's robot arm. It contains two parts, the basic part that consists of moving the arm up and down in succession, and an extra part that performs circular movements with the arm. This project is available as a git repository.

```bash
git clone https://github.com/Jagoca98/humanoides.git
```

### `map.wbt`

This file contains the map generated by `webots-map-from-csv_R2021a.py` using the `assets/map1.csv` file. The CSV file is designed based on the author's surnames (**Godoy Calvo**), with the size of the X and Y dimensions being the length of the surnames.

<p align="center">
    <img src="assets/Webots-Map.png" width="80%" height="80%">
</p>

### `my_controller`

This folder contains the `my_controller.cpp` file, which specifies how the exploration should be done. :mag_right: The movement of the robot is purely reactive, the turtlebot will try to keep going straight as long as it does not detect that it is within `0.5m` of a wall with the LiDAR readings. If it detects that it is within the threshold, it will turn in an arbitrary direction about itself on the `Z-axis` :arrows_counterclockwise:. In addition, it has a random component that forces it to turn even if it can continue forward :game_die:. With this behaviour, possible local minimum are broken :muscle:.






