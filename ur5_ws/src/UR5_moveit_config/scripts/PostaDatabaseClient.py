import pymongo
import datetime
import json
import math

class PostaDatabase:

    def __init__(self):
        self.HOST = "mongodb://br-c-proc-004.clients.pg.com:27017"
        self.dbClient = pymongo.MongoClient(self.HOST)
        self.dbName = "SUDPostaCobot"
        self.traysCollection = "Trays"
        self.samplesCollection = "Samples"
        self.Log = ""

    def GetDatabase(self):
        return self.dbClient[self.dbName]

    def GetTrays(self, includeArchived = False):
        db = self.GetDatabase()
        cll = db[self.traysCollection]
        if(includeArchived):
            trays_data = cll.find({})
        else:
            trays_data = cll.find({'Status':{'$ne':4}})
        trays = []
        for tray in trays_data:
            trays.append(tray)
        return trays

    def GetTraysWithId(self, trayId):
        db = self.GetDatabase()
        cll = db[self.traysCollection]
        trays_data = cll.find({"AlphanumericCode":trayId})
        trays = []
        for tray in trays_data:
            trays.append(tray)
        return trays

    def GetActiveTrays(self):
        db = self.GetDatabase()
        cll = db[self.traysCollection]
        trays_data = cll.find({'Status':1})
        trays = []
        for tray in trays_data:
            trays.append(tray)
        return trays

    def GetSamples(self, sampleIds):
        db = self.GetDatabase()
        cll = db[self.samplesCollection]
        samples = []
        for sID in sampleIds:
            sample = cll.find({'_id':sID})
            if(sample is not None):
                for smp in sample:
                    samples.append((smp))
        return samples

    def UpdateSample(self, sample):
        db = self.GetDatabase()
        cll = db[self.samplesCollection]
        try:
            filter = {'_id':sample._id}
            update = {'$set':
                {
                    'AnalysisDate': sample.AnalysisDate, 
                    'Status': sample.Status,
                    'WeightGrams': sample.WeightGrams,
                    'PouchStrengthN': sample.PouchStrengthN,
                    'PouchTightnessMM': sample.PouchTightnessMM,
                    'Results': sample.Results,
                    'Log': sample.Log
                }}
            
            result = cll.update_one(filter, update)
            return result

        except Exception as e:
            self.Log = 'Failed to update sample, ex: {}'.format(e)
            return False

    def UpdateTray(self, tray):
        db = self.GetDatabase()
        cll = db[self.traysCollection]
        
        try:
            # build the edit part
            filter = {'_id':tray._id, }
            update =  { '$set': { 'Status': tray.Status}} 
            return cll.update_one(filter,update)

        except Exception as e:
            self.Log = 'EXCEPTION while updating tray: {}'.format(e)
            return False

class PostaSample:

    def __init__(self):
        self._id = None
        self.ProductId = ""
        self.ResultId = ""
        self.CreateDate = None
        self.AnalysisDate = None
        self.TrayRow = -1
        self.TrayColumn = -1
        self.Status = 0
        self.RequestedProcedures = 0
        self.Results = ""
        self.WeightGrams = 0
        self.PouchStrengthN = 0
        self.PouchTightnessMM = 0
        self.Log = ""
        
        # Define the location of the actual sample
        self.DetectedRectangle = None # [xUL, yUL, width, height] in mm
        self.PouchPresent = False
        self.PouchType = ""
        self.PouchProbability = 0

    def toJson(self):
        return json.dumps(self, default=lambda o:o.__dict__, sort_keys=True, indent=4)

    def parse(self, object):
        try:
            self._id = object['_id']
            self.ProductId = object['ProductId']
            self.ResultId = object['ResultId']
            self.CreateDate = object['CreateDate']
            self.AnalysisDate = object['AnalysisDate']
            self.TrayRow = object['TrayRow']
            self.TrayColumn = object['TrayColumn']
            self.Status = object['Status']
            self.Results = object['Results']
            if('RequestedProcedures' in object):
                self.RequestedProcedures = object['RequestedProcedures']
            self.WeightGrams = object['WeightGrams']
            self.PouchStrengthN = object['PouchStrengthN']
            self.PouchTightnessMM = object['PouchTightnessMM']
            self.Log = object['Log']
            return True
        except Exception as e:
            print('ERROR while parsing PostaSample: {}'.format(e))
            return False
    
    def Update(self):
        db = PostaDatabase()
        res = db.UpdateSample(self)
        if res.matched_count>0:
            return True
        else:
            self.Log = db.Log
            return False

    @property
    def pCenterMM(self):
        return [
            self.DetectedRectangle[0] + self.DetectedRectangle[2]/2,
            self.DetectedRectangle[1] + self.DetectedRectangle[3]/2
        ]

    @staticmethod
    def GetSamples(sampleIds):
        # retrieve all sample with the given ids
        db = PostaDatabase()
        org_samples = db.GetSamples(sampleIds)
        samples = []
        for oSmp in org_samples:
            sample = PostaSample()
            if(sample.parse(oSmp)):
                samples.append((sample))
        return samples


class PostaSampleTray:

    def __init__(self):
        self._id = None
        self.AlphanumericCode = ""
        self.Name = ""
        self.CreateDate = None
        self.Status = 0 # Created, Inqueue, Analyzed, Error (same for trays)
        self.Log = ""
        self.SampleIds = None
        self.Samples = None
        self.DetectedSamples = None

        # geometrical parameters of tray
        self.TrayULMM = [80,45] # mm distance of the tray UL from the M0 Ul corner
        self.TraySizeMM = [575.0,430.0]
        self.Rows = 8
        self.Columns = 10
        self.TrayPouchRectangleMM = [self.TraySizeMM[0]/self.Columns, self.TraySizeMM[1]/self.Rows]

    def get_target_rectangle_mm(self, row, column): # 0 based indexing
        r = [
            self.TrayULMM[0] + column * self.TrayPouchRectangleMM[0],
            self.TrayULMM[1] + row * self.TrayPouchRectangleMM[1]
        ]
        return r

    def get_slot_from_point(self, x, y):
        # find where the point falls into
        fcol = (x-self.TrayULMM[0])/self.TrayPouchRectangleMM[0]
        frow = (y-self.TrayULMM[1])/self.TrayPouchRectangleMM[1]
        column = math.floor(fcol) + 1
        row = math.floor(frow) + 1
        return row,column

    def parse(self, object):
        try:
            self._id = object['_id']
            self.AlphanumericCode = object['AlphanumericCode']
            self.Name = object['Name']
            self.CreateDate = object['CreateDate']
            self.Status = object['Status']
            self.SampleIds = object['SampleIds']
            #if(object['Samples'] is not None):
            #    for smp in object['Samples']:
            #        posta_smp = PostaSample()
            #        if(posta_smp.parse(smp)):
            #            self.Samples.append(posta_smp)
            return True
        except Exception as e:
            print('ERROR while parsing PostaTray: {}'.format(e))
            return False

    @property
    def NumberOfTargetSamples(self):
        if(self.Samples is not None):
            return len(self.Samples)
        else:
            return 0

    @property
    def NumberOfDetectedSamples(self):
        if(self.Samples is not None):
            dSmp = [smp for smp in self.Samples if(smp.PouchPresent==True)]
            if(dSmp is not None):
                return len(dSmp)
            else:
                return 0
        else:
            return 0

    def DownloadSamples(self):
        self.Samples = PostaSample.GetSamples(self.SampleIds)

    def import_detected_samples(self, det_samples):
        #pBox = BoundingBox()
        #pBox.Class = box.Class
        #pBox.probability = box.probability
        #pBox.xmin  
        #pBox.ymin 
        #pBox.xmax 
        #pBox.ymax 

        # clear the actual position for existing samples
        for smp in self.Samples:
            smp.DetectedRectangle = None             
            smp.PouchPresent = False
            smp.PouchType = ""
            smp.PouchProbability = 0

        # inspect the detected samples and link with targets
        nFound = 0
        cnt = 0
        self.DetectedSamples = []

        for det_smp in det_samples:
            cnt+=1
            # get the row and column
            pC = [
                (det_smp.xmax+det_smp.xmin)/2.0,
                (det_smp.ymax+det_smp.ymin)/2.0
            ]
            row, col = self.get_slot_from_point(pC[0],pC[1])
            # find the target sample
            tgt_samples = [smp for smp in self.Samples if (smp.TrayRow==row) & (smp.TrayColumn==col)]
            if (tgt_samples is not None) & len(tgt_samples)>0:
                if(len(tgt_samples)==1):
                    # register the details in this sample
                    tSmp = tgt_samples[0]
                    tSmp.DetectedRectangle = [det_smp.xmin, det_smp.ymin, (det_smp.xmax-det_smp.xmin), (det_smp.ymax-det_smp.ymin)]
                    tSmp.PouchPresent = True
                    tSmp.PouchType = det_smp.Class
                    tSmp.PouchProbability = det_smp.probability
                    nFound += 1
                    self.DetectedSamples.append(tSmp)
                else:
                    print('ERROR: found more than one sample for point {},{}'.format(pC[0], pC[1]))
        
        return nFound>0
        
    def get_next_pouch(self):
        # retrieve the next available pouch starting from top left, going for rows
        for rIdx in range(self.Rows):
            for cIdx in range(self.Columns):
                tgt_samples = [smp for smp in self.Samples if ((smp.TrayRow-1)==rIdx) & ((smp.TrayColumn-1)==cIdx)]
                if(len(tgt_samples)==1):
                    if(tgt_samples[0].PouchPresent):
                        return tgt_samples[0]
        return None

    def how_many_pouches_to_analyze(self):
        # retrieve the next available pouch not analyzed
        cnt = 0
        for rIdx in range(self.Rows):
            for cIdx in range(self.Columns):
                tgt_samples = [smp for smp in self.Samples if ((smp.TrayRow-1)==rIdx) & ((smp.TrayColumn-1)==cIdx)]
                if(len(tgt_samples)==1):
                    if(not(tgt_samples[0].Status==2)):
                        cnt += 1
        return cnt

    def Update(self):
        db = PostaDatabase()
        res = db.UpdateTray(self)
        if res.matched_count>0:
            return True
        else:
            self.Log = db.Log
            return False

    @staticmethod
    def GetAllTrays(includeArchived = False):
        # retrieve all active trays from database
        db = PostaDatabase()
        org_trays = db.GetTrays(includeArchived)
        trays = []
        for otray in org_trays:
            tray = PostaSampleTray()
            if (tray.parse(otray)):
                trays.append(tray)
        return trays

    @staticmethod
    def GetAllActiveTrays():
        # retrieve all active trays from database
        db = PostaDatabase()
        org_trays = db.GetTrays()
        trays = []
        for otray in org_trays:
            tray = PostaSampleTray()
            if (tray.parse(otray)) & (tray.Status == 1):
                trays.append(tray)
        return trays

    @staticmethod
    def GetTray(trayCode):
        # retrieve all active trays from database
        db = PostaDatabase()
        org_trays = db.GetTraysWithId(trayCode)
        trays = []
        for otray in org_trays:
            tray = PostaSampleTray()
            if (tray.parse(otray)):
                trays.append(tray)
        
        if(len(trays)>0):
            return trays[0]
        else:
            return None

    @staticmethod
    def GetUniqueActiveTray():
        # If there is only one active tray return it
        db = PostaDatabase()
        org_trays = db.GetActiveTrays()
        trays = []
        for otray in org_trays:
            tray = PostaSampleTray()
            if (tray.parse(otray)):
                # get samples for this tray
                tray.Samples = PostaSample.GetSamples(tray.SampleIds)

                trays.append(tray)
        
        if(len(trays)==1):
            return True, trays[0]
        else:
            return False, None


if __name__ == '__main__':
    
    #tray_present, tray = PostaSampleTray.GetUniqueActiveTray()
    trays = PostaSampleTray.GetAllTrays()

    tray = PostaSampleTray.GetTray('000B')
    samples = PostaSample.GetSamples((tray.SampleIds))

    if (samples is not None) & (len(samples)>0):
        
        # Modify the sample
        smp = samples[0]
        smp.WeightGrams = 12.2
        smp.PouchStrengthN = 650
        smp.PouchTightnessMM = 23.5
        smp.AnalysisDate = datetime.datetime.today()
        smp.Status = 2

        # Update sample data
        if(smp.Update()):
            print('Sample update successful!')
        else:
            print('Sample update failed: {}'.format(smp.Log))


        # Update tray data
        tray.Status = 2
        if(tray.Update()):
            print('Tray update successful!')
        else:
            print('Tray update failed: {}'.format(tray.Log))


        print('Tray retrieved: Acode:{} Name:{}'.format(tray.AlphanumericCode, tray.Name))

    else:
        print('Tray could not be found')