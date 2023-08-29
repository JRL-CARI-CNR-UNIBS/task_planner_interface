from pymongo import MongoClient

# Configura la connessione al database MongoDB
client = MongoClient()
db = client['iso15066_lun_31']  # Sostituisci 'nome_database' con il nome del tuo database
collection = db['task_results_online']

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
    if "RELAXED" in recipe:
        max_t_end = result['max_t_end']
        min_t_start = result['min_t_start']
        duration_difference = max_t_end - min_t_start
        print(f"Recipe: {recipe}, Duration Difference: {duration_difference}")
    
