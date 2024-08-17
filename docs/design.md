# Project Design Break-down

## RobUI Package
1. Render different components like textboxes, checkboxes, and buttons based on a configuration file.
    - The path of the configuration file should be passed as a parameter to the UI.
1. Contains an internal publisher.
1. Contains an internal subscriber.

## RobMove Package
1. Contains all logic needed for this project completely decoupled from the UI.
1. Contains an internal subscriber.
1. Contains an internal publisher.
1. Control behaviours for different robots based on a configuration file.
    - The path of the configuration file should be passed as a parameter to the node.

## ConfigGen Tool
1. Generate configuration files for the UI and RobMove packages.