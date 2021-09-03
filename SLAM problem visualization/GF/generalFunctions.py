import numpy as np

#_importedNoNumba_ = False
try:
    from GF.generalFunctionsPrecompiled import *
except:
    print("generalFunctionsPrecompiled not found, attemping to recreate...")
    try:
        import GF.generalFunctionsCompileExport as temp
        temp.compileAll()
        del temp
        from GF.generalFunctionsPrecompiled import *
    except Exception as excepOne:
        print("failed to recreate generalFunctionsPrecompiled:", excepOne)
        print("falling back to 'precompiled' njit...")
        try:
            from GF.generalFunctionsNjit import *
            compileAll()
            del compileAll
            #del njit
        except Exception as excepTwo:
            print("failed to fall back to 'precompiled' njit:", excepTwo)
            print("falling back to generalFunctionsNoNumba")
            try:
                from GF.generalFunctionsNoNumba import *
                #_importedNoNumba_ = True
            except Exception as excepThree:
                print("failed to fall back to generalFunctionsNoNumba:", excepThree)
                print("giving up :(")

# Array Scalar Multiplication and Addition (numpy probably also has these, but whatever)
global ASM, ASA
ASM = lambda scalar, inputArray : [scalar * entry for entry in inputArray] #numpy does this by default, but python lists don't
ASA = lambda scalar, inputArray : [scalar + entry for entry in inputArray] #numpy does this by default, but python lists don't
#intBoolInv = lambda inputInt : (0 if (inputInt>0) else 1) #treat an int like a bool and invert it

#finding things in lists
def findIndexBy2DEntry(listToSearch, indexToCompare, valueToFind): #finds matching entry in 2D lists and returns index
    """find the index (of the outer list) that has an entry at the entered index (of the inner list) equal to the entered value (basically, finding entries in 2D lists)"""
    for i in range(len(listToSearch)):
        if(listToSearch[i][indexToCompare] == valueToFind):
            return(i)
    return(-1) #not found

# def findIndexBy3DEntry(listToSearch: np.ndarray, firstIndexToCompare, secondIndexToCompare, valueToFind): #finds matching entry in 3D lists and returns index
#     """find the index (of the outer-most list) that has an entry at the entered indexes (of the 2nd and 3rd dimension lists) equal to the entered value (basically, finding entries in 3D lists)"""
#     for i in range(len(listToSearch)):
#         if(listToSearch[i][firstIndexToCompare][secondIndexToCompare] == valueToFind):
#             return(i)
#     return(-1) #not found

#finding things in lists of classes
def findIndexByClassAttr(listToSearch, attrName: str, valueToFind): #finds matching attribute in lists of class objects and returns index
    """find the index (in a list of class objects) where the attribute with the entered name equals the entered value"""
    for i in range(len(listToSearch)):
        if(getattr(listToSearch[i], attrName) == valueToFind):
            return(i)
    return(-1) #not found

def findMinAttrIndex(inputList: np.ndarray, attrName: str): #finds smallest attribute in 1D list of classes and returns index
    """find the index that holds the minimum (attribute) value, as well as that value"""
    if(len(inputList) > 0):
        returnIndex = 0
        minVal = getattr(inputList[0], attrName)
        for i in range(len(inputList)):
            val = getattr(inputList[i], attrName)
            if(val < minVal):
                minVal = val
                returnIndex = i
        return(returnIndex, minVal)
    else:
        return(-1, 0)

def findMaxAttrIndex(inputList, attrName: str): #finds biggest attribute in 1D list of classes and returns index
    """find the index that holds the maximum (attribute) value, as well as that value"""
    if(len(inputList) > 0):
        returnIndex = 0
        maxVal = getattr(inputList[0], attrName)
        for i in range(len(inputList)):
            val = getattr(inputList[i], attrName)
            if(val > maxVal):
                maxVal = val
                returnIndex = i
        return(returnIndex, maxVal)
    else:
        return(-1, 0)