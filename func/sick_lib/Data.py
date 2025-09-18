# -*- coding: utf-8 -*-
"""
This module handles the incoming data and extracts information from it.

Copyright note: Redistribution and use in source, with or without modification, are permitted.

created: 2015-01-19
updated: 2018-02-02

author: Uwe Hahne, Torben Peichl
SICK AG, Waldkirch
email: techsupport0905@sick.de
Last commit: $Date: 2019-08-15 10:32:49 +0200 (Do, 15 Aug 2019) $
Last editor: $Author: dedekst $

Version: "$Revision: 23518 $"

"""

from struct import unpack, calcsize
import xml.etree.cElementTree as ET
import logging

# ----------------------------------------------------------------------------------------------------------------------
MAX_CONFIDENCE = 65535
# ----------------------------------------------------------------------------------------------------------------------

class Data():
    """ Gathers methods to handle the raw data. """

    def __init__(self, xmlParser=None, changedCounter=-1, depthmap=None, polarData=None, checksum='E'):
        self.xmlParser = xmlParser
        self.changedCounter = changedCounter
        self.depthmap = depthmap
        self.polarData2D = polarData
        self.checksum = checksum
        self.corrupted = False

    def read(self, dataBuffer):
        """ Extracts necessary data segments and triggers parsing of segments. """

        # first 11 bytes contain some internal definitions
        tempBuffer = dataBuffer[0:11]
        (magicword, pkglength, protocolVersion, packetType) =  unpack('>IIHB', tempBuffer)
        assert (magicword == 0x02020202)
        logging.debug("Package length: %s", pkglength)
        logging.debug("Protocol version: %s", protocolVersion)  # expected to be == 1
        logging.debug("Packet type: %s", packetType)  # expected to be  == 98

        # next four bytes an id (should equal 1) and
        # the number of segments (should be 3)
        tempBuffer = dataBuffer[11:15]
        (segid, numSegments) = unpack('>HH', tempBuffer)
        logging.debug("Blob ID: %s", segid)  # expected to be == 1
        logging.debug("Number of segments: %s", numSegments)  # expected to be  == 3

        # offset and changedCounter, 4 bytes each per segment
        offset = [None] * numSegments
        changedCounter = [None] * numSegments
        tempBuffer = dataBuffer[15:15 + numSegments * 2 * 4]
        for i in range(numSegments):
            index = i * 8
            (offset[i], changedCounter[i]) = \
                unpack('>II', tempBuffer[index:index + 8])
            offset[i] += 11
        logging.debug("Offsets: %s", offset)  # offset in bytes for each segment
        logging.debug("Changed counter: %s", changedCounter)  # counter for changes in the data

        # first segment describes the data format in XML
        xmlSegment = dataBuffer[offset[0]:offset[1]]
        logging.debug("The whole XML segment:")
        logging.debug(xmlSegment)
        # second segment contains the binary data
        binarySegment = dataBuffer[offset[1]:offset[2]]

        if (numSegments == 3):
            overlaySegment = dataBuffer[offset[2]:pkglength+4+4] # numBytes(magicword) = 4, numBytes(pkglength) = 4
            logging.debug("The whole overlay XML segment:")
            logging.debug(overlaySegment)

        checksum = dataBuffer[pkglength+8]
        if checksum != ord(self.checksum):
          logging.error("Checksum is wrong: %s (expected %s)" % (checksum, self.checksum)) # checksum of whole data
          self.corrupted = True
        else:
          logging.debug("Checksum: %s", checksum) # checksum of whole data
          self.corrupted = False

        # parsing the XML in order to extract necessary image information
        # only parse if something has changed
        if (self.changedCounter < changedCounter[0]):
            logging.debug("XML did change, parsing started.")
            myXMLParser = XMLParser()
            myXMLParser.parse(xmlSegment)
            self.xmlParser = myXMLParser
            self.changedCounter = changedCounter[0]
        else:
            logging.debug("XML did not change, not parsing again.")
            myXMLParser = self.xmlParser

        myBinaryParser = BinaryParser()

        self.hasDepthMap = False
        self.hasPolar2D = False
        self.hasCartesian = False

        if myXMLParser.hasDepthMap:
            logging.debug("Data contains depth map, reading camera params")
            self.hasDepthMap = True
            self.cameraParams = \
                CameraParameters(width=myXMLParser.imageWidth,
                                 height=myXMLParser.imageHeight,
                                 cam2worldMatrix=myXMLParser.cam2worldMatrix,
                                 fx=myXMLParser.fx, fy=myXMLParser.fy,
                                 cx=myXMLParser.cx, cy=myXMLParser.cy,
                                 k1=myXMLParser.k1, k2=myXMLParser.k2,
                                 f2rc=myXMLParser.f2rc)

            # extracting data from the binary segment (distance, intensity
            # and confidence).
            if myXMLParser.stereo:
                numBytesDistance = myXMLParser.imageHeight * \
                                   myXMLParser.imageWidth * \
                                   myXMLParser.numBytesPerZValue
            else:
                numBytesDistance = myXMLParser.imageHeight * \
                                   myXMLParser.imageWidth * \
                                   myXMLParser.numBytesPerDistanceValue
            numBytesIntensity = myXMLParser.imageHeight * \
                                myXMLParser.imageWidth * \
                                myXMLParser.numBytesPerIntensityValue
            numBytesConfidence = myXMLParser.imageHeight * \
                                 myXMLParser.imageWidth * \
                                 myXMLParser.numBytesPerConfidenceValue

            try:
                numBytesFrameNumber = myXMLParser.numBytesFrameNumber
                numBytesQuality = myXMLParser.numBytesQuality
                numBytesStatus = myXMLParser.numBytesStatus
            except AttributeError:
                numBytesFrameNumber = 0
                numBytesQuality = 0
                numBytesStatus = 0

            logging.info("Reading binary segment...")
            myBinaryParser.getDepthMap(binarySegment,
                                       numBytesFrameNumber,
                                       numBytesQuality,
                                       numBytesStatus,
                                       numBytesDistance,
                                       numBytesIntensity,
                                       myXMLParser.numBytesPerIntensityValue,
                                       numBytesConfidence)
            logging.info("...done.")

            if myXMLParser.stereo:
                distance = list(myBinaryParser.depthmap.distance)
                for i in range(0, len(distance)):
                    distance[i] = distance[i] / 10.0 #account for sub-millimeter values
                myBinaryParser.depthmap.distance = tuple(distance)
            self.depthmap = myBinaryParser.depthmap

        if myXMLParser.hasPolar2DData:
            self.hasPolar2D = True
            if (myXMLParser.hasDepthMap):
                myBinaryParser.getPolar2D(myBinaryParser.remainingBuffer, myXMLParser.numPolarValues)
            else:
                myBinaryParser.getPolar2D(binarySegment, myXMLParser.numPolarValues)
            if hasattr(myBinaryParser, 'polardata'):
                self.polarData2D = myBinaryParser.polardata
            else:
                self.hasPolar2D = False
        elif myXMLParser.hasCartesianData:
            self.hasCartesian = True
            if (myXMLParser.hasDepthMap):
                myBinaryParser.getCartesian(myBinaryParser.remainingBuffer)
            else:
                myBinaryParser.getCartesian(binarySegment)
            if hasattr(myBinaryParser, 'cartesianData'):
                self.cartesianData = myBinaryParser.cartesianData
            else:
                self.hasCartesian = False


class XMLParser:
    """ The XML parser that only extracts the needed information. """
    
    def __init__(self):
        self.revision = None
        self.imageWidth = None
        self.imageHeight = None
        self.dataItems = []
        self.frameLength = None
        self.cam2worldMatrix = None
        self.fx = None
        self.fy = None
        self.cx = None
        self.cy = None
        self.k1 = None
        self.k2 = None
        self.f2rc = None
        self.availableFrames = None
        self.hasDepthMap = False
        self.hasCartesianData = False
        self.hasPolar2DData = False
    
    def getDataFormat(self, xmlNode):
        
        if xmlNode is None:
            raise RuntimeError("ERROR: no xmlNode is not set")
            
        self.dataItems = []
        #print ET.tostring(xmlNode)
        knownDepthMap = ["TimestampUTC", "Version"]
        knownDataStream = ["FrameNumber", "Quality", "Status", "Z", "Distance", "Intensity", "Confidence"]
        knownSizes = { "uint8":1, "uint16":2, "uint32":4, "uint64":8}
                
        for node in xmlNode.find('FormatDescriptionDepthMap'):
            if node.tag in knownDepthMap:
                if node.tag == "TimestampUTC":
                    actualSize = "uint64"
                else:
                    actualSize = node.text
                logging.debug("Adding data \"{}\" with byte length {}".format(node.tag, knownSizes[actualSize]))
                self.dataItems.append({"Name" : node.tag , "Size" : knownSizes[actualSize]})
        
        for node in xmlNode.find('FormatDescriptionDepthMap/DataStream'):
            if node.tag in knownDataStream:
                logging.debug("Adding data \"{}\" with byte length {}".format(node.tag, knownSizes[node.text]))
                self.dataItems.append({"Name" : node.tag , "Size" : knownSizes[node.text]})
    
    def calcFrameLength(self):
        
        if self.imageWidth is None:
            raise RuntimeError("ERROR: imageWidth is not set")
        
        if self.imageHeight is None:
            raise RuntimeError("ERROR: imageHeight is not set")
        
        knownMapNames = ["Z", "Distance", "Intensity", "Confidence"]
        
        self.frameLength = 0
        for item in self.dataItems:
            if item["Name"] in knownMapNames:
                self.frameLength += item["Size"] * self.imageWidth * self.imageHeight
            else:
                self.frameLength += item["Size"]
        
        logging.debug("Calculated framelength (from xml): {} bytes".format(self.frameLength))
    
    def parse(self, xmlString):
        """ Parse method needs the XML segment as string input. """

        sickRecord = ET.fromstring(xmlString)  # the whole block set
        # is called sickrecord
        self.hasDepthMap = False
        self.hasPolar2DData = False
        self.hasCartesianData = False
        # extract camera/image parameters directly from XML
        # =======================================================================
        
        logging.debug("Parse data items ...")
        
        # wildcard * should be either DataSetDepthMap or DataSetStereo
        self.getDataFormat(sickRecord.find('DataSets/*'))
        
        self.stereo = (sickRecord.find('DataSets/DataSetStereo') is not None)
        logging.info(" Blob-XML contains DataSets/DataSetStereo ? -> {}".format(self.stereo))

        for dataSetDepthMap in sickRecord.iter('DataSetStereo' if self.stereo else 'DataSetDepthMap'):
            self.availableFrames = int(dataSetDepthMap.get('datacount'))
            logging.debug("datacount: {}".format(self.availableFrames))
            self.binFileName = dataSetDepthMap.find('DataLink/FileName').text
            self.hasDepthMap = True

            self.f2rc = 0.0

            for formatDescriptionDepthMap in dataSetDepthMap.iter('FormatDescriptionDepthMap'):
                for width in formatDescriptionDepthMap.iter('Width'):
                    self.imageWidth = int(width.text)
                for height in formatDescriptionDepthMap.iter('Height'):
                    self.imageHeight = int(height.text)
                for cameraToWorldTransform in formatDescriptionDepthMap.iter('CameraToWorldTransform'):
                    self.cam2worldMatrix = []  # init array
                    for idx, child in enumerate(cameraToWorldTransform):
                        self.cam2worldMatrix.append(float(child.text))  # fill array row by row (4x4 matrix)
                for cameraMatrix in formatDescriptionDepthMap.iter('CameraMatrix'):  # camera intrinsics (fx,fy,cx,cy) are the sixth child in datastream
                    self.fx = float(cameraMatrix.find('FX').text)
                    self.fy = float(cameraMatrix.find('FY').text)
                    self.cx = float(cameraMatrix.find('CX').text)
                    self.cy = float(cameraMatrix.find('CY').text)
                for distortionParams in formatDescriptionDepthMap.iter('CameraDistortionParams'):
                    self.k1 = float(distortionParams.find('K1').text)  # we only need k1 and k2
                    self.k2 = float(distortionParams.find('K2').text)
                for f2rc in formatDescriptionDepthMap.iter('FocalToRayCross'):
                    self.f2rc = float(f2rc.text)
                for frameNumber in formatDescriptionDepthMap.iter('FrameNumber'):
                    if frameNumber.text.lower() == 'uint32':
                        self.numBytesFrameNumber = 4
                for quality in formatDescriptionDepthMap.iter('Quality'):
                    if quality.text.lower() == 'uint8':
                        self.numBytesQuality = 1
                for status in formatDescriptionDepthMap.iter('Status'):
                    if status.text.lower() == 'uint8':
                        self.numBytesStatus = 1
                if self.stereo:
                    for Z in formatDescriptionDepthMap.iter('Z'):
                         self.distType = Z.text
                         if Z.text.lower() == 'uint16':
                             self.numBytesPerZValue = 2
                             self.numBytesPerDistanceValue = 2 # leagacy for ssr-loader
                else:
                    for distance in formatDescriptionDepthMap.iter('Distance'):
                        self.distType = distance.text
                        if distance.text.lower() == 'uint16':
                            self.numBytesPerDistanceValue = 2
                for intensity in formatDescriptionDepthMap.iter('Intensity'):
                    self.intsType = intensity.text
                    if intensity.text.lower() == 'uint16':
                        self.numBytesPerIntensityValue = 2
                    elif intensity.text.lower() == 'uint32':
                        self.numBytesPerIntensityValue = 4
                for confidence in formatDescriptionDepthMap.iter('Confidence'):
                    self.confType = confidence.text
                    if confidence.text.lower() == 'uint16':
                        self.numBytesPerConfidenceValue = 2
        self.calcFrameLength()
        # =======================================================================
        for dataSetPolar2D in sickRecord.iter('DataSetPolar2D'):
            self.hasPolar2DData = True
            for formatDescription in dataSetPolar2D.iter('FormatDescription'):
                for dataStream in formatDescription.iter('DataStream'):
                    if dataStream.attrib.get('type') == 'distance':
                        self.numPolarValues = int(dataStream.attrib.get('datalength'))
        # =======================================================================
        for dataSetCartesian in sickRecord.iter('DataSetCartesian'):
            self.hasCartesianData = True
            for formatDescriptionCartesian in dataSetCartesian.iter('FormatDescriptionCartesian'):
                for dataStream in formatDescriptionCartesian.iter('DataStream'):
                    for length in dataStream.iter('Length'):
                        assert(length.text.lower() == 'uint32')
                    for x in dataStream.iter('X'):
                        assert(x.text.lower() == 'float32')
                    for y in dataStream.iter('Y'):
                        assert(y.text.lower() == 'float32')
                    for z in dataStream.iter('Z'):
                        assert(z.text.lower() == 'float32')
                    for confidence in dataStream.iter('Intensity'): # 'Intensity is not a typo, we abuse this element in the format for confidence values
                        assert(confidence.text.lower() == 'float32')



class BinaryParser:
    """ The binary parser for extracting distance, intensity and confidence from
    the binary segment of the raw data frame.
    """

    def getDepthMap(self,
                    binarySegment,
                    numBytesFrameNumber,
                    numBytesQuality,
                    numBytesStatus,
                    numBytesDistance,
                    numBytesIntensity,
                    numBytesPerIntensityValue,
                    numBytesConfidence):
        position = 0
        # the binary part starts with entries for length, a timestamp
        # and a version identifier
        infoBlockSize = calcsize('<IQH')
        (lengthAtStart, timeStamp, version) = unpack('<IQH', binarySegment[position:position + infoBlockSize])
        position += infoBlockSize
        logging.debug("Length at start: %s", lengthAtStart)
        self.logTimeStamp(timeStamp)
        logging.debug("Format version: %s", version)

        format2BlockSize = 0
        if version == 2:
            assert numBytesFrameNumber == 4
            assert numBytesQuality == 1
            assert numBytesStatus == 1
            format2BlockSize = calcsize('<IBB')
            (frameNumber, quality, status) = unpack('<IBB', binarySegment[position:position + format2BlockSize])
            position += format2BlockSize
            logging.debug("FrameNumber: %s", frameNumber)
            logging.debug("Data quality: %s", quality)
            logging.debug("Device status: %s", status)
        else:
            logging.warning("Old format, no values for frameNumber, quality and status")
            frameNumber = -1
            quality = 0
            status = 0

        dataBlockSize = numBytesDistance + \
                        numBytesIntensity + \
                        numBytesConfidence  # calculating the end index
        dataBinary = binarySegment[position:position + dataBlockSize]  # whole data block
        position += dataBlockSize
        distance = dataBinary[0:numBytesDistance]  # only the distance data (as string)

        logging.debug("Reading distance...")
        distanceData = unpack('<%uH' % (len(distance) / 2), distance)
        logging.debug("...done.")

        # extract the intensity data (same procedure as distance)
        logging.debug("Reading intensity...")
        off = numBytesDistance
        intensity = dataBinary[off:numBytesIntensity + off]
        if numBytesPerIntensityValue == 2:
            intensityData = unpack('<%uH' % (len(intensity) / 2), intensity)
        elif numBytesPerIntensityValue == 4:
            intensityData = unpack('<%uL' % (len(intensity) / 4), intensity)
        else:
            # legacy mode, also used for RGBA -> byte-wise
            intensityData = unpack('<%uB' % len(intensity), intensity)
        logging.debug("...done.")

        # extract the confidence data (same procedure as distance)
        logging.debug("Reading confidence...")
        off += numBytesIntensity
        confidence = dataBinary[off:numBytesConfidence + off]
        confidenceData = unpack('<%uH' % (len(confidence) / 2), confidence)
        logging.debug("...done.")

        # checking if all data is read
        if (position + 4 == lengthAtStart):
            check = calcsize('<II')
            (crc, lengthAtEnd) = unpack('<II', binarySegment[position:position + check])
            position += check
            logging.debug("Length at start: %s", lengthAtStart)
            logging.debug("Length at end: %s", lengthAtEnd)
            #assert lengthAtStart == lengthAtEnd
            if lengthAtStart != lengthAtEnd:
                logging.error("lengthAtStart != lengthAtEnd")
        self.remainingBuffer = binarySegment[position:]

        self.depthmap = DepthMap(distanceData, intensityData, confidenceData, frameNumber, quality, status, timeStamp)

    def getPolar2D(self,
                   binarySegment,
                   numPolarValues):
        position = 0
        infoBlockSize = calcsize('<IQHIIffffff')
        if len(binarySegment) < infoBlockSize:
            logging.warning("Found inconsitency in binary polar data.")
            return
        (lengthAtStart, timeStamp, deviceID, scanCounter, syscountScan, scanFrequency, measFrequency, angleFirstScanPoint, angularResolution, scale,
         offset) = unpack('<IQHIIffffff', binarySegment[position:position + infoBlockSize])
        logging.debug("Length = %i" % lengthAtStart)
        position += infoBlockSize

        self.logTimeStamp(timeStamp)
        logging.debug("DeviceID = %i" % deviceID)
        logging.debug("ScanCounter = %i" % scanCounter)
        logging.debug("SyscountScan = %i" % syscountScan)
        logging.debug("ScanFrequency = %i" % scanFrequency)
        logging.debug("MeasFrequency = %i" % measFrequency)
        logging.debug("AngleFirstScanPoint = %i" % angleFirstScanPoint)
        logging.debug("AngularResolution = %i" % angularResolution)
        logging.debug("Scale = %i" % scale)
        logging.debug("Offset = %i" % offset)

        logging.debug("Reading position is now: %i" % position)
        # distanceDataSize = int(singleValueSize * numPolarValues)
        endPosition = position + numPolarValues * calcsize('<f')
        if len(binarySegment) < endPosition:
            logging.warning("Found inconsitency in binary polar data.")
            return
        distanceData = unpack('<%uf' % numPolarValues, binarySegment[position:endPosition])
        logging.debug("Distance data = %s" % str(distanceData))
        position = endPosition
        logging.debug("Reading position is now: %i" % position)

        confidenceBlockSize = calcsize('<ffff')
        endPosition = position + confidenceBlockSize
        if len(binarySegment) < endPosition:
            logging.warning("Found inconsitency in binary polar data.")
            return
        (rssi_startAngle, rssi_angularResolution, rssi_scale, rssi_offset) = unpack('<ffff', binarySegment[position:endPosition])
        logging.debug("RSSI AngleFirstScanPoint = %i" % rssi_startAngle)
        logging.debug("RSSI AngularResolution = %i" % rssi_angularResolution)
        logging.debug("RSSI Scale = %i" % rssi_scale)
        logging.debug("RSSI Offset = %i" % rssi_offset)

        position = endPosition
        logging.debug("Reading position is now: %i" % position)

        endPosition = position + numPolarValues * calcsize('<f')
        if len(binarySegment) < endPosition:
            logging.warning("Found inconsitency in binary polar data.")
            return
        confidenceData = unpack('<%uf' % numPolarValues, binarySegment[position:endPosition])
        logging.debug("Confidence data = %s" % str(confidenceData))
        # convert into percent if needed
        # confidence = tuple((val / MAX_CONFIDENCE) * 100.0 for val in confidenceData)
        position = endPosition
        logging.debug("Reading position is now: %i" % position)

        # checking if all data is read
        if (position + 8 == lengthAtStart):
            check = calcsize('<II')
            endPosition = position + check
            if len(binarySegment) < endPosition:
                logging.warning("Found inconsitency in binary polar data.")
                return
            (crc, lengthAtEnd) = unpack('<II', binarySegment[position:endPosition])
            logging.debug("Length at start: %s", lengthAtStart)
            logging.debug("Length at end: %s", lengthAtEnd)
            assert lengthAtStart == lengthAtEnd
            self.hasRemainingBuffer = False
        else:
            # there is another data set in this binary buffer
            self.hasRemainingBuffer = True
            self.remainingBuffer = binarySegment[position:lengthAtStart + 4]
            self.length = lengthAtStart  # checking if all data is read

        self.polardata = Polar2DData(angleFirstScanPoint, angularResolution, distanceData, confidenceData, timeStamp)

    def getCartesian(self,
                   binarySegment):
        position = 0
        infoBlockSize = calcsize('<IQHI')
        if len(binarySegment) < infoBlockSize:
            logging.warning("Found inconsitency in binary polar data.")
            return
        (lengthAtStart, timeStamp, version, numPoints) = unpack('<IQHI', binarySegment[position:position + infoBlockSize])
        logging.debug("Length = %i" % lengthAtStart)
        position += infoBlockSize

        self.logTimeStamp(timeStamp)
        logging.debug("Version = %i" % version)

        logging.debug("Reading position is now: %i" % position)

        endPosition = position + numPoints * calcsize('<ffff') # 4 float32 values per point
        if len(binarySegment) < endPosition:
            logging.warning("Found inconsitency in binary polar data.")
            return
        pointCloudData = unpack('<%uf' % (numPoints*4), binarySegment[position:endPosition])
        x = pointCloudData[0:numPoints*4:4]
        y = pointCloudData[1:numPoints*4:4]
        z = pointCloudData[2:numPoints*4:4]
        rssi = pointCloudData[3:numPoints*4:4]
        confidence = tuple((val / MAX_CONFIDENCE) * 100.0 for val in rssi)

        position = endPosition
        logging.debug("Reading position is now: %i" % position)
        logging.debug("Point cloud data = %s" % str(pointCloudData))

        # checking if all data is read
        check = calcsize('<II')
        endPosition = position + check
        if len(binarySegment) < endPosition:
            logging.warning("Found inconsitency in binary Cartesian data.")
            return
        (crc, lengthAtEnd) = unpack('<II', binarySegment[position:endPosition])
        logging.debug("Length at start: %s", lengthAtStart)
        logging.debug("Length at end: %s", lengthAtEnd)
        assert lengthAtStart == lengthAtEnd
        self.hasRemainingBuffer = False

        self.cartesianData = CartesianData(numPoints, x, y, z, confidence, timeStamp)

    # ===============================================================================

    def logTimeStamp(self, timeStamp):
        # 0x03 D9 08 40 02 C7 B0 00
        # 0000 0011 1101 1001 0000 1000 0100 0000 0000 0010 1100 0111 1011 0000 0000 0000
        # .... .YYY YYYY YYYY YMMM MDDD DDTT TTTT TTTT THHH HHMM MMMM SSSS SSmm mmmm mmmm
        # Bits: 5 unused - 12 Year - 4 Month - 5 Day - 11 Timezone - 5 Hour - 6 Minute - 6 Seconds - 10 Milliseconds
        # .....YYYYYYYYYYYYMMMMDDDDDTTTTTTTTTTTHHHHHMMMMMMSSSSSSmmmmmmmmmm
        YearMask         = 0b0000011111111111100000000000000000000000000000000000000000000000
        MonthMask        = 0b0000000000000000011110000000000000000000000000000000000000000000
        DayMask          = 0b0000000000000000000001111100000000000000000000000000000000000000
        TimezoneMask     = 0b0000000000000000000000000011111111111000000000000000000000000000
        HourMask         = 0b0000000000000000000000000000000000000111110000000000000000000000
        MinuteMask       = 0b0000000000000000000000000000000000000000001111110000000000000000
        SecondsMask      = 0b0000000000000000000000000000000000000000000000001111110000000000
        MillisecondsMask = 0b0000000000000000000000000000000000000000000000000000001111111111
        Year = (timeStamp & YearMask) >> 47
        Month = (timeStamp & MonthMask) >> 43
        Day = (timeStamp & DayMask) >> 38
        Timezone = (timeStamp & TimezoneMask) >> 27
        Hour = (timeStamp & HourMask) >> 22
        Minute = (timeStamp & MinuteMask) >> 16
        Seconds = (timeStamp & SecondsMask) >> 10
        Milliseconds = timeStamp & MillisecondsMask
        logging.debug("Data Timestamp [YYYY-MM-DD HH:MM:SS.mm] = %04u-%02u-%02u %02u:%02u:%02u.%03u" % (Year, Month, Day, Hour, Minute, Seconds, Milliseconds))


class CameraParameters:
    """ This class gathers the main camera parameters. """

    def __init__(self, width=176, height=144,
                 cam2worldMatrix=[],
                 fx=146.5, fy=146.5, cx=84.4, cy=71.2,
                 k1=0.326442, k2=0.219623,
                 f2rc=0.0):
        self.width = width
        self.height = height
        self.cam2worldMatrix = [1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1]
        self.fx = fx
        self.fy = fy
        self.cx = cx
        self.cy = cy
        self.k1 = k1
        self.k2 = k2
        self.f2rc = f2rc


class DepthMap():
    """ This class contains the depthmap data """

    def __init__(self, distance, intensity, confidence, frameNumber, dataQuality, deviceStatus, timestamp):
        self.distance = distance
        self.intensity = intensity
        self.confidence = confidence
        self.frameNumber = frameNumber
        self.dataQuality = dataQuality
        self.deviceStatus = deviceStatus
        self.timestamp = timestamp

class Polar2DData():
    """ This class contains the polar 2D data """

    def __init__(self, angleFirstScanPoint, angularResolution, distance, confidence, timestamp):
        self.distance = distance
        self.angleFirstScanPoint = angleFirstScanPoint
        self.angularResolution = angularResolution
        self.confidence = confidence
        self.timestamp = timestamp

class CartesianData():
    """ This class contains the polar 2D data """

    def __init__(self, numPoints, x, y, z, confidence, timestamp):
        self.numPoints = numPoints
        self.x = x
        self.y = y
        self.z = z
        self.confidence = confidence
        self.timestamp = timestamp
