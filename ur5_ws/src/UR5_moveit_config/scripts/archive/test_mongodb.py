import pymongo
import datetime

myClient = pymongo.MongoClient("mongodb://br-c-proc-004.clients.pg.com:27017")
#print(myClient.list_database_names())

myDb = myClient["EmmaAI_01"]
projColl = myDb["Projects"]
imgColl = myDb["OriginalImages"]
PROJECT_NAME = "SUD Posta Pouches"

query = {"Name":PROJECT_NAME}
print(query)
sud_proj = projColl.find_one(query)

if sud_proj is not None:
    print(sud_proj)
    proj_id = sud_proj["_id"]
    print('Found project id: ',proj_id)

    # get one image
    qImg = {"ProjectID":proj_id}
    img = imgColl.find_one(qImg)

    if img is not None:
        print('Image width: ', img["Width"])

    # try inserting a new image...
    del img["_id"]
    img["UserName"] = "Cobot TEST"
    img["UploadDate"] = datetime.datetime.utcnow()
    inserted_image = imgColl.insert_one(img)

    if(inserted_image is not None):
        print(inserted_image.inserted_id)
    else:
        print('Inser image FAILED!')

    


