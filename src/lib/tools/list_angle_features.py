from distutils.log import ERROR
import pymongo

NODE_INDICES = [0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16, 17, 18, 19, 20, 21, 22, 23]
NODE_NAME = ["M_Hip", "L_Hip", "R_Hip", "L_Back", "L_Knee", "R_Knee", "M_Back", "L_Ankle", "R_Ankle", "U_Back", "L_Toes", "R_Toes", "Neck", "L_Collarbone", "R_Collarbone", "Head", "L_Shoulder", "R_Shoulder", "L_Elbow", "R_Elbow", "L_Wrist", "R_Wrist", "L_Fingers", "R_Fingers"]

def main():
    mongo_client = pymongo.MongoClient("mongodb://mongoadmin:secret@localhost:27888/?authSource=admin", serverSelectionTimeoutMS=2000)
    if mongo_client is None:
        return False
    db = mongo_client.trainerai
    exercises = db.exercises
    x = exercises.find()

    for data in x:
        try:
            print(f'### {data["description"]} ###')
            features = data["features"]
            angles = []
            distances = []
            for feature in features:
                values = ""
                for value in feature["value"]:
                    index = NODE_INDICES[NODE_NAME.index(str(value))]
                    values += f"{index} "
                if feature["type"] == "angle":
                    angles.append(values)
                elif feature["type"] == "distance":
                    distances.append(values)
            print("--- Angles ---")
            for val in angles:
                print(f"{val}")
            print("--- Distance ---")
            for val in distances:
                print(f"{val}")

            print()
        except ERROR as err:
            print(err)

if __name__ == "__main__":
    main()