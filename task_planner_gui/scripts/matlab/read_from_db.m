
close all; clc


javaaddpath('/usr/share/java/mysql-connector-java-8.0.20.jar');
conn = database('data_task_planner','root','password','Vendor','MySQL', 'Server', 'localhost', 'PortNumber', 3306);

% %Select all tasks from all sessions
% selectquery = 'SELECT * FROM task';
% data = select(conn,selectquery)


% %Select all tasks where session = 28
% selectquery = 'SELECT * FROM task WHERE session_id = 28';
% data = select(conn,selectquery)


%Select all tasks where task name = "Pick"
selectquery = 'SELECT * FROM task WHERE name = "Pick"';
data = select(conn,selectquery)


