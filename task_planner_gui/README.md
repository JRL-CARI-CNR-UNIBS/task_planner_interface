The package provides a visual GUI to improve user's interaction with the planner. The GUI is based on the [Qt](#https://www.qt.io/), [Qt-Quick](#https://doc.qt.io/qt-5/qtquick-index.html) library, so latest packages will be required. More details about dependancies are listed in section [2 -Prerequisite](#prerequisite).

## Dependencies

Update to the latest version of [Qt](www.qt.io). Then, run in a terminal the following commands to install additional modules:

```

sudo apt update

sudo apt install build-essential

sudo apt install cmake 

sudo apt install qtbase5-dev

sudo apt install qtdeclarative5-dev

sudo apt install qtquickcontrols2-5-dev 

sudo apt install qml-module-qtquick2

sudo apt install qml-module-qtquick-controls

sudo apt install qml-module-qtquick-controls2  

sudo apt install mysql-client

sudo apt install libmysqlclient-dev

sudo apt install libqt5sql5-mysql

sudo apt install libmysql++-dev

sudo apt install libqt5charts5-dev 

sudo apt install qml-module-qtcharts

```

In order to use **data analysis** and **database** features, it's mandatory to install the latest version of [MySQL DBMS](https://dev.mysql.com/doc/mysql-getting-started/en/)  setting the correct user/password. To install MySql, type in a new terminal:

```

sudo apt update

sudo apt install mysql-server

```

Then, open MySQL:

```

sudo mysql

```

MySQL-window will appear. Now, define a new connection with *user* **root** and related *password* by typing :

```

ALTER USER 'root'@'localhost' IDENTIFIED WITH mysql_native_password BY 'password';

FLUSH PRIVILEGES;

exit

```

The main *user* is now **root** and the new *password* is just: '**password**'.

(**IMPORTANT:** *user* and *password* must be the ones specified above, do not change them for other reasons).



Now, load the *database* by typing in a terminal:

```

roscd task_planner_gui

sudo mysql -u root -p < scripts/db/data_task_planner_db.sql 

```

Insert the *password* specified. Finally, test if the database *data_task_planner_db* appears in the list of databases by connecting to MySQL server by typing:

```

sudo mysql -u root -p

```

And list all databases:

```

SHOW DATABASES;

```

Quit the connection by typing *quit*. Well, all done! The database is configured.



**Additional tips:**

If you want to better visualize your database, it is possible to download **MySQL Workbench**. In this way, all your connections, database, tables etc. will be displayed in a convenient UI:

```

sudo apt install mysql-workbench

```

## Example

To run the gui launch:

```

roslaunch task_planner_gui gui.launch

```
The launcher runs the node **task_interface_gui**:
Topics:
```
- /sharework/test/stiima/gui/human/feedback # gui publishes task feedback to planner
- /sharework/test/stiima/motion/feedback # gui receives feedback of robot task to update visual
- /sharework/test/stiima/human # gui receives and show request of task for human
- /sharework/test/stiima/motion # gui recevies and show request of task for robot
```

##  Visualization
Once started, the interface will look as follow:
![](task_planner_gui/qt_src/assets/images/gui_task_planner3.png) 

For each task, the GUI visualized in transparent box data of:
```
string cmd_name            # command type. examples: pick, place, pickplace, move
string cfg_start           # symbolic identifcation of the initial task position
string cfg_goal            # symbolic identifcation of the task goal
```
Multiple columns divide task between human and robot:

- *Robot 1*: new tasks published on topic *topic_motion_request (Robot)* appear here. Each task wil automatically scroll up-down in real time depending on the **expected_time** of execution. The **expected_time** is visualized on screen.
A vertical bar visualized data of **risk_level** :
```
float64 risk_level        # (0.0 < r < 0.33 *GREEN*, 0.33 < r < 0.66 *YELLOW*, 0.66 < r < 1.0 *RED*>)
```

The GUI will run in a node parallel to the **robot_node** (or human) to support data visualization, letting lowe level nodes manage the backend code implementation. Each low level can work even in the absence of **task_interface_gui** node, ensuring modularity to the framework.

### Data Analysis Visualization

The following figure shows the *data analysis* tab:
![](task_planner_interface/qt_src/assets/images/gui_task_planner4_modified.png)
For each task it displays data related to:

- **Time** to complete selected task
- Average **human-robot distance**
- Average robot **override**

All tasks are collected in **Sessions**. A new Session is created every time you start the GUI .

The **heat diagrams** display a possibile risk level for each task as a correlation between task completion time and robot override (or HR-distance).
*For instance*: the *green area* in the first diagram is related to low task execution time and low robot override. It means that a task has been executed fast and with the robot moving slow, thus, a very low risk level for the user.

### Data Flows
The following schema shows the typical nodes comunications:
![](task_planner_interface/qt_src/assets/images/gui_task_planner2.png) 

The low level human_node waits for that message before sending feedback to the task planner. If no topic is provided, the human_node will not wait to send feedback at the task planner.


### Matlab Data Import
*NOTE:* Before using Matlab features to operate with SQL DB, complete all steps from [Section 2.2 - GUI Database dependencies](#guidbdep).

It is possibile to define any desired SQL queries for a database from a Matlab script. First, it is necessary to install a *Connnector* and related *JDBC Drivers* to access (and connect) to a SQL DB from Matlab. Download the latest drivers [here](https://dev.mysql.com/downloads/connector/j/). They will be automatically installed in:
```
/usr/share/java/mysql-connector-java-8.0.20.jar
```
To test simple queries for the DB, an example script is located in:
```
task_planner_interface/scripts/matlab/read_from_db.m
```
*NOTE:* if the drivers install path is different, specify the new path in *read_from_db.m* at the line:
```
javaaddpath('PATH_TO_DRIVERS_CONNECTOR.jar');
```

