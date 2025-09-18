"""
全局变量模块，用于线程/前后端间的数据传递
"""
import json


def init():
    """
    初始化方法，仅在入口最顶层调用一次。创建配置参数和公共变量所使用的字典
        :return: 无
    """
    global GlobalParam  # 配置参数，该字典内容可被保存在配置文件中
    global GlobalValue  # 公共变量，该字典内容不会保存
    GlobalParam = {}
    GlobalValue = {}


def saveToFile(default=False):
    """
    写入配置参数到文件
        :param default: 是否写入到默认配置文件
        :return: 无
    """
    # 根据标志选择配置文件路径
    if default:
        cfg_path = './DefaultSettings.cfg'
    else:
        cfg_path = './Settings.cfg'
    with open(cfg_path, 'w', encoding='utf-8') as cfg_obj:
        cfg_obj.write(json.dumps(getAll(), ensure_ascii=False, indent=4, separators=(',', ':')))


def loadFromFile(default=False):
    """
    从文件载入全局变量
        :param default: 是否载入默认配置
        :return: 无
    """
    if default:
        with open('./DefaultSettings.cfg', 'r', encoding='utf-8') as file_obj:
            loaded = json.load(file_obj)
    else:
        with open('./Settings.cfg', 'r', encoding='utf-8') as file_obj:
            loaded = json.load(file_obj)
    # 写入公共变量
    setParam('baggageAlpha', float(loaded['baggageAlpha']))
    setParam('baggageBeta', float(loaded['baggageBeta']))
    setParam('baggageGamma', float(loaded['baggageGamma']))
    setParam('baggageXmin', float(loaded['baggageXmin']))
    setParam('baggageYmin', float(loaded['baggageYmin']))
    setParam('baggageZmin', float(loaded['baggageZmin']))
    setParam('baggageXmax', float(loaded['baggageXmax']))
    setParam('baggageYmax', float(loaded['baggageYmax']))
    setParam('baggageZmax', float(loaded['baggageZmax']))
    setParam('baggageMinPts', int(loaded['baggageMinPts']))
    setParam('baggageRadius', float(loaded['baggageRadius']))
    setParam('baggageZOffset', float(loaded['baggageZOffset']))
    setParam('baggageEps', float(loaded['baggageEps']))
    setParam('baggageStd', float(loaded['baggageStd']))
    setParam('baggagePts', int(loaded['baggagePts']))

    setParam('stackAlpha', float(loaded['stackAlpha']))
    setParam('stackBeta', float(loaded['stackBeta']))
    setParam('stackGamma', float(loaded['stackGamma']))
    setParam('stackXmin', float(loaded['stackXmin']))
    setParam('stackYmin', float(loaded['stackYmin']))
    setParam('stackZmin', float(loaded['stackZmin']))
    setParam('stackXmax', float(loaded['stackXmax']))
    setParam('stackYmax', float(loaded['stackYmax']))
    setParam('stackZmax', float(loaded['stackZmax']))
    setParam('stackMinPts', int(loaded['stackMinPts']))
    setParam('stackRadius', float(loaded['stackRadius']))
    setParam('stackZOffset', float(loaded['stackZOffset']))
    setParam('stackEps', float(loaded['stackEps']))
    setParam('stackStd', float(loaded['stackStd']))
    setParam('stackPts', int(loaded['stackPts']))

    setParam('pickIncrementX', float(loaded['pickIncrementX']))
    setParam('putIncrementX', float(loaded['putIncrementX']))
    setParam('putIncrementY', float(loaded['putIncrementY']))
    setParam('putIncrementZ', float(loaded['putIncrementZ']))

    setParam('hostIP', str(loaded['hostIP']))
    setParam('hostPort', int(loaded['hostPort']))
    setParam('stackCameraIP', str(loaded['stackCameraIP']))
    setParam('stackCameraPort', int(loaded['stackCameraPort']))

    setParam('modelContinuous', bool(loaded['modelContinuous']))
    setParam('stackModelPath', str(loaded['stackModelPath']))
    setParam('gridX', int(loaded['gridX']))
    setParam('gridY', int(loaded['gridY']))
    setParam('gridZ', int(loaded['gridZ']))
    setParam('scaleX', float(loaded['scaleX']))
    setParam('scaleY', float(loaded['scaleY']))
    setParam('scaleZ', float(loaded['scaleZ']))
    setParam('baggageLengthMin', int(loaded['baggageLengthMin']))
    setParam('baggageLengthMax', int(loaded['baggageLengthMax']))
    setParam('baggageLengthIncrement', float(loaded['baggageLengthIncrement']))
    setParam('baggageWidthMin', int(loaded['baggageWidthMin']))
    setParam('baggageWidthMax', int(loaded['baggageWidthMax']))
    setParam('baggageWidthIncrement', float(loaded['baggageWidthIncrement']))
    setParam('baggageThickMin', int(loaded['baggageThickMin']))
    setParam('baggageThickMax', int(loaded['baggageThickMax']))
    setParam('baggageThickIncrement', float(loaded['baggageThickIncrement']))
    setParam('stackLengthIncrement', float(loaded['stackLengthIncrement']))
    setParam('stackWidthIncrement', float(loaded['stackWidthIncrement']))
    setParam('stackThickIncrement', float(loaded['stackThickIncrement']))
    setParam('leafNode', int(loaded['leafNode']))
    setParam('internalNode', int(loaded['internalNode']))


    setParam('classifyModelPath', str(loaded['classifyModelPath']))
    setParam('baggageImgCropXmin', float(loaded['baggageImgCropXmin']))
    setParam('baggageImgCropXmax', float(loaded['baggageImgCropXmax']))
    setParam('baggageImgCropYmin', float(loaded['baggageImgCropYmin']))
    setParam('baggageImgCropYmax', float(loaded['baggageImgCropYmax']))
    setParam('baggageImageSize', int(loaded['baggageImageSize']))


def setParam(name, value):
    """
    为指定的配置参数设定值，该方法设定的值会保存在配置文件中
        :param name: 配置参数名，详见手册附录
        :param value: 配置参数的值
        :return: 设置是否成功
    """
    try:
        GlobalParam[name] = value
        return True
    except KeyError:
        return False


def getParam(name):
    """
    从配置参数中取值
        :param name: 配置参数名，详见手册附录
        :return: 对应的配置参数值
    """
    try:
        return GlobalParam[name]
    except KeyError:
        return None


def setValue(name, value):
    """
    为指定的公共变量设定值，该方法设定的值不会保存在配置文件中
        :param name: 公共变量名，详见手册附录
        :param value: 公共变量的值
        :return: 设置是否成功
    """
    try:
        GlobalValue[name] = value
        return True
    except KeyError:
        return False


def getValue(name):
    """
    从公共变量中取值
        :param name: 公共变量名，详见手册附录
        :return: 对应的公共变量值
    """
    try:
        return GlobalValue[name]
    except KeyError:
        return None


def getAll():
    """
    获取整个配置参数字典
        :return: 包含所有参数的字典
    """
    return GlobalParam
