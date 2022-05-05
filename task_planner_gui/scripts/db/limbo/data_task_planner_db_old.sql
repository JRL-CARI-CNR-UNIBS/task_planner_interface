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
) ENGINE=InnoDB AUTO_INCREMENT=118 DEFAULT CHARSET=latin1;
/*!40101 SET character_set_client = @saved_cs_client */;

--
-- Dumping data for table `sessions`
--

LOCK TABLES `sessions` WRITE;
/*!40000 ALTER TABLE `sessions` DISABLE KEYS */;
INSERT INTO `sessions` VALUES (26,'mar mag 5 2020','15:20:49'),(28,'mar mag 5 2020','15:32:34'),(29,'mar mag 5 2020','15:40:15'),(30,'mar mag 5 2020','15:44:50'),(31,'mar mag 5 2020','15:59:40'),(32,'mar mag 5 2020','16:01:37'),(33,'mar mag 5 2020','16:02:51'),(34,'mar mag 5 2020','16:04:09'),(35,'mar mag 5 2020','16:04:33'),(36,'mar mag 5 2020','16:05:42'),(37,'mar mag 5 2020','16:06:01'),(38,'mar mag 5 2020','16:09:43'),(39,'mar mag 5 2020','16:13:07'),(40,'mar mag 5 2020','16:32:28'),(41,'mar mag 5 2020','16:33:25'),(42,'mar mag 5 2020','16:37:05'),(43,'mar mag 5 2020','16:37:59'),(44,'mar mag 5 2020','16:39:54'),(45,'mar mag 5 2020','16:40:55'),(46,'mar mag 5 2020','16:43:26'),(47,'mar mag 5 2020','17:06:39'),(48,'mar mag 5 2020','17:08:14'),(49,'mar mag 5 2020','17:09:25'),(50,'mar mag 5 2020','17:10:59'),(51,'mar mag 5 2020','17:12:23'),(52,'mar mag 5 2020','17:12:59'),(53,'mar mag 5 2020','17:14:50'),(54,'mar mag 5 2020','17:15:21'),(55,'mar mag 5 2020','17:15:42'),(56,'mar mag 5 2020','17:16:30'),(57,'mar mag 5 2020','17:21:13'),(58,'mar mag 5 2020','17:22:32'),(59,'mar mag 5 2020','17:23:31'),(60,'mar mag 5 2020','17:23:57'),(61,'mar mag 5 2020','17:24:23'),(62,'mar mag 5 2020','17:25:22'),(63,'mar mag 5 2020','17:25:52'),(64,'mar mag 5 2020','17:28:14'),(65,'mar mag 5 2020','17:29:03'),(66,'mar mag 5 2020','17:30:03'),(67,'mar mag 5 2020','17:57:34'),(68,'mar mag 5 2020','18:33:07'),(69,'mar mag 5 2020','18:33:35'),(70,'mar mag 5 2020','18:51:28'),(71,'mar mag 5 2020','19:00:48'),(72,'mar mag 5 2020','19:01:28'),(73,'mar mag 5 2020','19:22:13'),(74,'mar mag 5 2020','19:22:52'),(75,'mar mag 5 2020','19:23:12'),(76,'mar mag 5 2020','19:25:06'),(77,'mar mag 5 2020','19:35:16'),(78,'mar mag 5 2020','19:35:45'),(79,'mar mag 5 2020','21:28:42'),(80,'mar mag 5 2020','21:29:01'),(81,'mar mag 5 2020','21:29:07'),(82,'mar mag 5 2020','21:29:22'),(83,'mar mag 5 2020','21:30:19'),(84,'mar mag 5 2020','21:31:06'),(85,'mar mag 5 2020','21:32:19'),(86,'mar mag 5 2020','21:32:56'),(90,'lun mag 11 2020','19:44:04'),(91,'lun mag 11 2020','19:47:50'),(92,'lun mag 11 2020','19:48:56'),(93,'lun mag 11 2020','19:55:25'),(94,'lun mag 11 2020','20:02:54'),(95,'lun mag 11 2020','20:04:25'),(96,'lun mag 11 2020','20:04:56'),(97,'lun mag 11 2020','23:11:06'),(98,'lun mag 11 2020','23:32:24'),(99,'lun mag 11 2020','23:33:31'),(100,'lun mag 11 2020','23:34:43'),(101,'lun mag 11 2020','23:36:10'),(102,'lun mag 11 2020','23:36:51'),(103,'lun mag 11 2020','23:47:23'),(104,'lun mag 11 2020','23:50:28'),(105,'lun mag 11 2020','23:52:08'),(106,'lun mag 11 2020','23:53:07'),(107,'lun mag 11 2020','23:53:41'),(108,'lun mag 11 2020','23:54:07'),(109,'lun mag 11 2020','23:54:41'),(110,'lun mag 11 2020','23:55:01'),(111,'lun mag 11 2020','23:55:38'),(112,'lun mag 11 2020','23:55:52'),(113,'lun mag 11 2020','23:56:36'),(114,'lun mag 11 2020','23:57:26'),(115,'lun mag 11 2020','23:58:07'),(116,'mar mag 12 2020','00:02:17'),(117,'mar mag 12 2020','01:04:25');
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
  `robot` varchar(45) DEFAULT NULL,
  `session_id` int(11) NOT NULL,
  `user_id` int(11) NOT NULL,
  `robot_id` int(11) DEFAULT NULL,
  PRIMARY KEY (`id`),
  KEY `fk_task_1_idx` (`session_id`),
  KEY `fk_task_2_idx` (`user_id`),
  CONSTRAINT `fk_task_1` FOREIGN KEY (`session_id`) REFERENCES `sessions` (`id`) ON DELETE CASCADE ON UPDATE NO ACTION,
  CONSTRAINT `fk_task_2` FOREIGN KEY (`user_id`) REFERENCES `users` (`id`) ON DELETE CASCADE ON UPDATE NO ACTION
) ENGINE=InnoDB AUTO_INCREMENT=92 DEFAULT CHARSET=latin1;
/*!40101 SET character_set_client = @saved_cs_client */;

--
-- Dumping data for table `task`
--

LOCK TABLES `task` WRITE;
/*!40000 ALTER TABLE `task` DISABLE KEYS */;
INSERT INTO `task` VALUES (77,'id_1','Pick','','',1.49,'SUCCESS',58.00,NULL,NULL,28,2,NULL),(78,'id_3','Assemble','','',1.73,'SUCCESS',76.00,NULL,NULL,28,2,NULL),(79,'id_3','Assemble','','',0.00,'REJECTED',73.00,NULL,NULL,28,2,NULL),(80,'id_1','Pick','','',2.45,'SUCCESS',56.00,NULL,NULL,63,2,NULL),(81,'id_3','Assemble','','',1.16,'SUCCESS',93.00,NULL,NULL,63,2,NULL),(82,'id_1','Pick','','',6.17,'SUCCESS',68.00,NULL,NULL,75,2,NULL),(83,'id_1','Pick','','',1.44,'SUCCESS',25.00,NULL,NULL,76,2,NULL),(84,'id_2','Place','','',1.82,'SUCCESS',9.00,NULL,NULL,76,2,NULL),(85,'id_3','Assemble','','',1.31,'SUCCESS',53.00,NULL,NULL,76,2,NULL),(86,'id_4','Complete','','',1.57,'SUCCESS',96.00,NULL,NULL,76,2,NULL),(87,'id_5','Finish','','',4.58,'SUCCESS',21.00,NULL,NULL,76,2,NULL),(88,'id_4','Complete','','',2.20,'SUCCESS',25.00,30.00,NULL,92,2,NULL),(89,'id_3','Assemble','','',2.55,'SUCCESS',5.00,83.00,NULL,92,2,NULL),(90,'id_2','Place','','',1.63,'SUCCESS',11.00,71.00,NULL,92,2,NULL),(91,'id_1','Pick','','',1.25,'SUCCESS',32.00,76.00,NULL,92,2,NULL);
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
) ENGINE=InnoDB AUTO_INCREMENT=3 DEFAULT CHARSET=latin1;
/*!40101 SET character_set_client = @saved_cs_client */;

--
-- Dumping data for table `users`
--

LOCK TABLES `users` WRITE;
/*!40000 ALTER TABLE `users` DISABLE KEYS */;
INSERT INTO `users` VALUES (2,'user1');
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

-- Dump completed on 2020-05-12  2:57:44
