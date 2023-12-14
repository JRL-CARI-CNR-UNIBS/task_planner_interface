from pymongo import MongoClient

import rospy

# Configura la connessione al database MongoDB
client = MongoClient()
db = client['hrc_case_study']  # Sostituisci 'nome_database' con il nome del tuo database
collection = db['real_test_results_test_finished']

# Esegui la query per raggruppare i risultati per "recipe"
pipeline = [
    {
        '$group': {
            '_id': '$recipe',
            'max_t_end': {'$max': '$t_end'},
            'min_t_start': {'$min': '$t_start'}
        }
    }
]

results = collection.aggregate(pipeline)

# Calcola la differenza tra max(t_end) e min(t_start) per ogni gruppo
for result in results:
    recipe = result['_id']
    if "RELAXED_HA_SOLVER" not in recipe:
        continue
    if ("10_26_11" not in recipe) and ("10_26_10" not in recipe):
        continue
    if "_" in recipe:
        max_t_end = result['max_t_end']
        min_t_start = result['min_t_start']
        duration_difference = max_t_end - min_t_start
        print(f"Recipe: {recipe}, Duration Difference: {duration_difference}")
