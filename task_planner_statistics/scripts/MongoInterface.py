#! /usr/bin/env python3

import rospy
from dataclasses import dataclass, field

from pymongo import MongoClient
import pymongo.errors

GREEN = '\033[92m'
YELLOW = '\033[93m'
RED = '\033[91m'
BOLD = '\033[1m'
END = '\033[0m'

CONNECTION_LOST = RED + "Connection to Database lost" + END


@dataclass
class MongoInterface:
    database_name: str
    # collection_properties_name: str
    # collection_results_name: str
    # collection_durations_name: str
    # collection_synergies_name: str

    def __post_init__(self):
        client = MongoClient(serverSelectionTimeoutMS=5000)  # 5 seconds of maximum connection wait

        if self.database_name not in client.list_database_names():
            rospy.loginfo(RED + "The specified db does not exist. A db with empty collections will be created. " + END)
            raise Exception

        self.db = client[self.database_name]

        # collections_name = [self.collection_properties_name,
        #                     self.collection_results_name,
        #                     self.collection_durations_name,
        #                     self.collection_synergies_name]
        # for collection_name in collections_name:
        #     if collection_name not in self.db.list_collection_names():
        #         rospy.loginfo(RED + f"Collection {collection_name} is not in database" + END)
        #         raise Exception(f"Collection {collection_name} is not in database")
        #
        # self.collection_properties = self.db[self.collection_properties_name]
        # self.collection_results = self.db[self.collection_results_name]
        # self.collection_durations = self.db[self.collection_durations_name]
        # self.collection_synergies = self.db[self.collection_synergies_name]

    def collection_exist(self,collection_name):
        if collection_name not in self.db.list_collection_names():
            return False
        return True

    def query(self, collection_name, pipeline):
        if not self.collection_exist(collection_name):
            raise Exception(f"Collection {collection_name} is not in database")

        try:
            result = self.db[collection_name].aggregate(pipeline)
        except pymongo.errors.AutoReconnect:
            rospy.logerr(CONNECTION_LOST)
            raise Exception(CONNECTION_LOST)
        return result

