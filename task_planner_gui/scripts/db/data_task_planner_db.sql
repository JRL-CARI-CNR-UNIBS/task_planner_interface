CREATE DATABASE  IF NOT EXISTS `data_task_planner_db` /*!40100 DEFAULT CHARACTER SET latin1 */;
USE `data_task_planner_db`;
-- MySQL dump 10.13  Distrib 5.7.30, for Linux (x86_64)
--
-- Host: 127.0.0.1    Database: data_task_planner_db
-- ------------------------------------------------------
-- Server version	5.7.30-0ubuntu0.18.04.1

/*!40101 SET @OLD_CHARACTER_SET_CLIENT=@@CHARACTER_SET_CLIENT */;
/*!40101 SET @OLD_CHARACTER_SET_RESULTS=@@CHARACTER_SET_RESULTS */;
/*!40101 SET @OLD_COLLATION_CONNECTION=@@COLLATION_CONNECTION */;
/*!40101 SET NAMES utf8 */;
/*!40103 SET @OLD_TIME_ZONE=@@TIME_ZONE */;
/*!40103 SET TIME_ZONE='+00:00' */;
/*!40014 SET @OLD_UNIQUE_CHECKS=@@UNIQUE_CHECKS, UNIQUE_CHECKS=0 */;
/*!40014 SET @OLD_FOREIGN_KEY_CHECKS=@@FOREIGN_KEY_CHECKS, FOREIGN_KEY_CHECKS=0 */;
/*!40101 SET @OLD_SQL_MODE=@@SQL_MODE, SQL_MODE='NO_AUTO_VALUE_ON_ZERO' */;
/*!40111 SET @OLD_SQL_NOTES=@@SQL_NOTES, SQL_NOTES=0 */;

--
-- Table structure for table `robots`
--

DROP TABLE IF EXISTS `robots`;
/*!40101 SET @saved_cs_client     = @@character_set_client */;
/*!40101 SET character_set_client = utf8 */;
CREATE TABLE `robots` (
  `id` int(11) NOT NULL AUTO_INCREMENT,
  `name` varchar(45) NOT NULL,
  PRIMARY KEY (`id`)
) ENGINE=InnoDB AUTO_INCREMENT=2 DEFAULT CHARSET=latin1;
/*!40101 SET character_set_client = @saved_cs_client */;

--
-- Dumping data for table `robots`
--

LOCK TABLES `robots` WRITE;
/*!40000 ALTER TABLE `robots` DISABLE KEYS */;
INSERT INTO `robots` VALUES (1,'ur5');
/*!40000 ALTER TABLE `robots` ENABLE KEYS */;
UNLOCK TABLES;

--
-- Table structure for table `sessions`
--

DROP TABLE IF EXISTS `sessions`;
/*!40101 SET @saved_cs_client     = @@character_set_client */;
/*!40101 SET character_set_client = utf8 */;
CREATE TABLE `sessions` (
  `id` int(11) NOT NULL AUTO_INCREMENT,
  `date` varchar(45) DEFAULT NULL,
  `clock` varchar(45) DEFAULT NULL,
  PRIMARY KEY (`id`)
) ENGINE=InnoDB AUTO_INCREMENT=183 DEFAULT CHARSET=latin1;
/*!40101 SET character_set_client = @saved_cs_client */;

--
-- Dumping data for table `sessions`
--

LOCK TABLES `sessions` WRITE;
/*!40000 ALTER TABLE `sessions` DISABLE KEYS */;
INSERT INTO `sessions` VALUES (26,'mar mag 5 2020','15:20:49'),(28,'mar mag 5 2020','15:32:34'),(29,'mar mag 5 2020','15:40:15'),(30,'mar mag 5 2020','15:44:50'),(31,'mar mag 5 2020','15:59:40'),(32,'mar mag 5 2020','16:01:37'),(33,'mar mag 5 2020','16:02:51'),(34,'mar mag 5 2020','16:04:09'),(35,'mar mag 5 2020','16:04:33'),(36,'mar mag 5 2020','16:05:42'),(37,'mar mag 5 2020','16:06:01'),(38,'mar mag 5 2020','16:09:43'),(39,'mar mag 5 2020','16:13:07'),(40,'mar mag 5 2020','16:32:28'),(41,'mar mag 5 2020','16:33:25'),(42,'mar mag 5 2020','16:37:05'),(43,'mar mag 5 2020','16:37:59'),(44,'mar mag 5 2020','16:39:54'),(45,'mar mag 5 2020','16:40:55'),(46,'mar mag 5 2020','16:43:26'),(47,'mar mag 5 2020','17:06:39'),(48,'mar mag 5 2020','17:08:14'),(49,'mar mag 5 2020','17:09:25'),(50,'mar mag 5 2020','17:10:59'),(51,'mar mag 5 2020','17:12:23'),(52,'mar mag 5 2020','17:12:59'),(53,'mar mag 5 2020','17:14:50'),(54,'mar mag 5 2020','17:15:21'),(55,'mar mag 5 2020','17:15:42'),(56,'mar mag 5 2020','17:16:30'),(57,'mar mag 5 2020','17:21:13'),(58,'mar mag 5 2020','17:22:32'),(59,'mar mag 5 2020','17:23:31'),(60,'mar mag 5 2020','17:23:57'),(61,'mar mag 5 2020','17:24:23'),(62,'mar mag 5 2020','17:25:22'),(63,'mar mag 5 2020','17:25:52'),(64,'mar mag 5 2020','17:28:14'),(65,'mar mag 5 2020','17:29:03'),(66,'mar mag 5 2020','17:30:03'),(67,'mar mag 5 2020','17:57:34'),(68,'mar mag 5 2020','18:33:07'),(69,'mar mag 5 2020','18:33:35'),(70,'mar mag 5 2020','18:51:28'),(71,'mar mag 5 2020','19:00:48'),(72,'mar mag 5 2020','19:01:28'),(73,'mar mag 5 2020','19:22:13'),(74,'mar mag 5 2020','19:22:52'),(75,'mar mag 5 2020','19:23:12'),(76,'mar mag 5 2020','19:25:06'),(77,'mar mag 5 2020','19:35:16'),(78,'mar mag 5 2020','19:35:45'),(79,'mar mag 5 2020','21:28:42'),(80,'mar mag 5 2020','21:29:01'),(81,'mar mag 5 2020','21:29:07'),(82,'mar mag 5 2020','21:29:22'),(83,'mar mag 5 2020','21:30:19'),(84,'mar mag 5 2020','21:31:06'),(85,'mar mag 5 2020','21:32:19'),(86,'mar mag 5 2020','21:32:56'),(92,'lun mag 11 2020','19:48:56'),(116,'mar mag 12 2020','00:02:17'),(117,'mar mag 12 2020','01:04:25'),(118,'lun mag 18 2020','17:44:15'),(119,'lun mag 18 2020','18:01:27'),(123,'mar mag 19 2020','12:34:48'),(124,'mar mag 19 2020','16:03:55'),(125,'mar mag 19 2020','16:06:28'),(127,'mar mag 19 2020','19:39:36'),(128,'mer mag 20 2020','11:33:51'),(129,'gio mag 21 2020','16:41:36'),(130,'gio mag 21 2020','18:06:02'),(131,'gio mag 21 2020','18:08:57'),(132,'gio mag 21 2020','18:12:23'),(133,'gio mag 21 2020','18:14:41'),(134,'gio mag 21 2020','18:15:42'),(135,'gio mag 21 2020','18:18:18'),(136,'gio mag 21 2020','18:19:01'),(137,'gio mag 21 2020','18:21:09'),(138,'gio mag 21 2020','18:31:48'),(139,'gio mag 21 2020','18:52:19'),(140,'gio mag 21 2020','18:53:39'),(142,'gio mag 21 2020','18:56:04'),(143,'gio mag 21 2020','19:10:00'),(144,'gio mag 21 2020','19:12:57'),(145,'gio mag 21 2020','19:14:58'),(146,'gio mag 21 2020','19:16:29'),(147,'gio mag 21 2020','19:17:21'),(148,'gio mag 21 2020','19:20:14'),(149,'gio mag 21 2020','19:20:49'),(150,'gio mag 21 2020','19:21:34'),(151,'gio mag 21 2020','19:22:49'),(152,'gio mag 21 2020','19:26:10'),(153,'gio mag 21 2020','19:27:05'),(154,'gio mag 21 2020','19:33:04'),(155,'gio mag 21 2020','19:34:52'),(156,'gio mag 21 2020','19:36:42'),(157,'gio mag 21 2020','19:38:29'),(158,'gio mag 21 2020','19:41:06'),(159,'gio mag 21 2020','19:41:41'),(160,'gio mag 21 2020','19:43:27'),(161,'ven mag 22 2020','11:48:03'),(162,'ven mag 22 2020','11:57:17'),(163,'ven mag 22 2020','12:20:11'),(164,'ven mag 22 2020','12:24:41'),(165,'ven mag 22 2020','12:24:58'),(166,'ven mag 22 2020','12:47:34'),(167,'ven mag 22 2020','12:50:22'),(168,'ven mag 22 2020','12:53:10'),(169,'ven mag 22 2020','13:03:45'),(170,'ven mag 22 2020','13:06:54'),(171,'ven mag 22 2020','13:09:17'),(172,'ven mag 22 2020','13:11:07'),(173,'ven mag 22 2020','13:11:55'),(174,'ven mag 22 2020','13:12:13'),(175,'ven mag 22 2020','13:12:33'),(176,'ven mag 22 2020','13:15:22'),(177,'ven mag 22 2020','13:16:27'),(178,'ven mag 22 2020','13:16:54'),(179,'ven mag 22 2020','13:17:14'),(180,'ven mag 22 2020','13:18:30'),(181,'ven mag 22 2020','13:18:58'),(182,'ven mag 22 2020','13:20:05');
/*!40000 ALTER TABLE `sessions` ENABLE KEYS */;
UNLOCK TABLES;

--
-- Table structure for table `task`
--

DROP TABLE IF EXISTS `task`;
/*!40101 SET @saved_cs_client     = @@character_set_client */;
/*!40101 SET character_set_client = utf8 */;
CREATE TABLE `task` (
  `id` int(11) NOT NULL AUTO_INCREMENT,
  `task_id` varchar(45) DEFAULT NULL,
  `name` varchar(45) DEFAULT NULL,
  `start` varchar(45) DEFAULT NULL,
  `goal` varchar(45) DEFAULT NULL,
  `time` decimal(4,2) DEFAULT NULL,
  `status` varchar(45) DEFAULT NULL,
  `distance_hr` decimal(4,2) DEFAULT NULL,
  `override_robot` decimal(4,2) DEFAULT NULL,
  `session_id` int(11) NOT NULL,
  `user_id` int(11) DEFAULT NULL,
  `robot_id` int(11) NOT NULL,
  PRIMARY KEY (`id`),
  KEY `fk_task_1_idx` (`session_id`),
  KEY `fk_task_2_idx` (`user_id`),
  CONSTRAINT `fk_task_1` FOREIGN KEY (`session_id`) REFERENCES `sessions` (`id`) ON DELETE CASCADE ON UPDATE NO ACTION,
  CONSTRAINT `fk_task_2` FOREIGN KEY (`user_id`) REFERENCES `users` (`id`) ON DELETE CASCADE ON UPDATE NO ACTION
) ENGINE=InnoDB AUTO_INCREMENT=197 DEFAULT CHARSET=latin1;
/*!40101 SET character_set_client = @saved_cs_client */;

--
-- Dumping data for table `task`
--

LOCK TABLES `task` WRITE;
/*!40000 ALTER TABLE `task` DISABLE KEYS */;
INSERT INTO `task` VALUES (107,'','Pick','','',0.51,'SUCCESS',4.00,3.00,138,NULL,1),(108,'','Pick','','',15.52,'SUCCESS',79.00,11.00,140,NULL,1),(110,'','Pick','','',3.22,'SUCCESS',11.00,2.00,142,NULL,1),(111,'','Pick','','',3.23,'SUCCESS',50.00,73.00,142,NULL,1),(112,'','Pick','','',3.24,'SUCCESS',14.00,78.00,142,NULL,1),(140,'','Pick','','',15.51,'SUCCESS',53.00,49.00,143,NULL,1),(141,'','Place','','',26.76,'SUCCESS',90.00,44.00,143,NULL,1),(142,'','Place','','',26.78,'SUCCESS',21.00,48.00,143,NULL,1),(143,'','Place','','',5.79,'SUCCESS',72.00,79.00,143,NULL,1),(144,'','Place','','',5.80,'SUCCESS',18.00,8.00,143,NULL,1),(145,'','Place','','',5.81,'SUCCESS',70.00,71.00,143,NULL,1),(146,'','Pick','','',15.54,'SUCCESS',64.00,1.00,144,NULL,1),(147,'','Place','','',22.56,'SUCCESS',73.00,98.00,144,NULL,1),(148,'','Place','','',22.57,'SUCCESS',82.00,99.00,144,NULL,1),(149,'','Place','','',22.58,'SUCCESS',19.00,89.00,144,NULL,1),(150,'','Pick','','',15.53,'SUCCESS',14.00,83.00,145,NULL,1),(151,'','Place','','',17.54,'SUCCESS',25.00,35.00,145,NULL,1),(152,'','Pick','','',5.21,'SUCCESS',77.00,16.00,146,NULL,1),(153,'','Pick','','',5.22,'SUCCESS',67.00,93.00,146,NULL,1),(154,'','Pick','','',5.23,'SUCCESS',81.00,32.00,146,NULL,1),(155,'','Pick','','',15.54,'SUCCESS',70.00,71.00,149,NULL,1),(156,'','Place','','',22.65,'SUCCESS',53.00,67.00,149,NULL,1),(157,'','Pick','','',15.52,'SUCCESS',65.00,40.00,150,NULL,1),(158,'','Place','','',23.76,'SUCCESS',2.00,54.00,150,NULL,1),(159,'','Place','','',12.72,'SUCCESS',78.00,58.00,150,NULL,1),(160,'','Place','','',20.25,'SUCCESS',18.00,37.00,151,NULL,1),(161,'','Pick','','',15.52,'SUCCESS',80.00,12.00,153,NULL,1),(162,'','Place','','',6.02,'SUCCESS',91.00,86.00,153,NULL,1),(163,'','Assemble','','',16.52,'SUCCESS',16.00,87.00,153,NULL,1),(164,'','Pick','','',4.42,'SUCCESS',92.00,84.00,154,NULL,1),(165,'','Place','','',3.32,'SUCCESS',8.00,67.00,154,NULL,1),(166,'','Assemble','','',5.52,'SUCCESS',14.00,3.00,154,NULL,1),(167,'','Assemble','','',16.51,'SUCCESS',25.00,28.00,155,NULL,1),(168,'','Pick','','',6.93,'SUCCESS',92.00,28.00,156,NULL,1),(169,'id_1','Pick','','',5.81,'SUCCESS',79.00,23.00,157,NULL,1),(170,'id_2','Place','','',3.32,'SUCCESS',60.00,37.00,157,NULL,1),(171,'id_3','Assemble','','',2.51,'SUCCESS',34.00,73.00,157,NULL,1),(172,'id_3','Assemble','','',4.52,'SUCCESS',76.00,42.00,158,NULL,1),(173,'id_2','Place','','',7.03,'SUCCESS',84.00,28.00,158,NULL,1),(174,'id_1','Pick','','',5.31,'SUCCESS',60.00,2.00,159,NULL,1),(175,'id_2','Place','','',5.33,'SUCCESS',55.00,15.00,159,NULL,1),(176,'id_1','Pick','','',15.51,'SUCCESS',78.00,97.00,161,NULL,1),(177,'id_1','Pick','','',15.52,'SUCCESS',54.00,88.00,161,NULL,1),(178,'id_2','Place','','',2.92,'SUCCESS',73.00,67.00,161,NULL,1),(179,'id_2','Place','','',17.24,'SUCCESS',52.00,52.00,162,NULL,1),(181,'id_2','Place','','',4.51,'SUCCESS',0.00,0.00,164,NULL,1),(182,'id_2','Place','','',10.33,'SUCCESS',5.94,0.00,165,NULL,1),(183,'id_1','Pick','','',6.32,'SUCCESS',7.00,0.00,165,NULL,1),(184,'id_1','Pick','','',7.13,'SUCCESS',6.17,0.00,166,NULL,1),(185,'id_2','Place','','',10.12,'SUCCESS',5.53,0.00,166,NULL,1),(186,'id_1','Pick','','',6.93,'SUCCESS',5.33,0.00,167,NULL,1),(187,'id_2','Place','','',8.92,'SUCCESS',5.45,0.00,167,NULL,1),(188,'id_2','Place','','',11.94,'SUCCESS',3.11,0.00,168,NULL,1),(189,'id_1','Pick','','',6.53,'SUCCESS',2.68,0.00,168,NULL,1),(190,'id_1','Pick','','',6.22,'SUCCESS',3.16,0.00,169,NULL,1),(191,'id_2','Place','','',6.92,'SUCCESS',2.83,0.00,169,NULL,1),(192,'id_2','Place','','',5.12,'SUCCESS',2.00,0.00,170,NULL,1),(193,'id_1','Pick','','',6.92,'SUCCESS',4.00,0.00,175,NULL,1),(194,'id_1','Pick','','',6.12,'SUCCESS',4.00,0.00,181,NULL,1),(195,'id_1','Pick','','',5.12,'SUCCESS',0.00,0.00,182,NULL,1),(196,'id_2','Place','','',7.52,'SUCCESS',2.00,0.00,182,NULL,1);
/*!40000 ALTER TABLE `task` ENABLE KEYS */;
UNLOCK TABLES;

--
-- Table structure for table `users`
--

DROP TABLE IF EXISTS `users`;
/*!40101 SET @saved_cs_client     = @@character_set_client */;
/*!40101 SET character_set_client = utf8 */;
CREATE TABLE `users` (
  `id` int(11) NOT NULL AUTO_INCREMENT,
  `name` varchar(45) NOT NULL,
  PRIMARY KEY (`id`)
) ENGINE=InnoDB AUTO_INCREMENT=4 DEFAULT CHARSET=latin1;
/*!40101 SET character_set_client = @saved_cs_client */;

--
-- Dumping data for table `users`
--

LOCK TABLES `users` WRITE;
/*!40000 ALTER TABLE `users` DISABLE KEYS */;
INSERT INTO `users` VALUES (2,'user1'),(3,'ur5');
/*!40000 ALTER TABLE `users` ENABLE KEYS */;
UNLOCK TABLES;
/*!40103 SET TIME_ZONE=@OLD_TIME_ZONE */;

/*!40101 SET SQL_MODE=@OLD_SQL_MODE */;
/*!40014 SET FOREIGN_KEY_CHECKS=@OLD_FOREIGN_KEY_CHECKS */;
/*!40014 SET UNIQUE_CHECKS=@OLD_UNIQUE_CHECKS */;
/*!40101 SET CHARACTER_SET_CLIENT=@OLD_CHARACTER_SET_CLIENT */;
/*!40101 SET CHARACTER_SET_RESULTS=@OLD_CHARACTER_SET_RESULTS */;
/*!40101 SET COLLATION_CONNECTION=@OLD_COLLATION_CONNECTION */;
/*!40111 SET SQL_NOTES=@OLD_SQL_NOTES */;

-- Dump completed on 2020-05-22 19:00:05
