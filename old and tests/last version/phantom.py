## use with caution

class phantomList(list):
    """(indexTable version) a list where you can set (list[index] = value) without the index existing, and read any (previously set) indices (even if they exceed len())"""
    def __init__(self, initList=None, eqIndexTable=None):
        self._eqIndexTable_ = list() #a list of equivalent indices
        if(type(initList) is phantomList):
            list.__init__(self, initList)
            self._eqIndexTable_ = initList._eqIndexTable_.copy()
        elif((initList is not None) and (eqIndexTable is not None)):
            if(hasattr(initList, '__iter__') and hasattr(initList, '__len__') and \
               hasattr(eqIndexTable, '__iter__') and hasattr(eqIndexTable, '__len__')): #check if initList and eqIndexTable are itterable (list/tuple/etc.)
                if(len(initList) == len(eqIndexTable)):
                    list.__init__(self, initList)
                    self._eqIndexTable_ = [item for item in eqIndexTable] #copy (as list()) eqIndexTable
                else:
                    raise(ValueError("phantomList init list & eqIndexTable length mismatch: "+str(len(initList))+" != "+str(len(eqIndexTable))))
            else:
                raise(TypeError("phantomList init list/eqIndexTable invalid types: "+str(type(initList))+" and "+str(type(eqIndexTable))))
        else:
            if(initList is not None):
                print("can't init phantomList() with:", type(initList), initList, eqIndexTable)
                raise(TypeError("phantomList init list invalid type (or missing eqIndexTable): "+str(type(initList))+" and "+str(type(eqIndexTable))))
            list.__init__(self)
    
    def __repr__(self):
        return("phantomList"+super(self.__class__, self).__repr__())
    
    def _findLocalIndex_(self, eqIndex):
        #return(self._eqIndexTable_.index(eqIndex))
        for i in range(len(self._eqIndexTable_)): #i'm assuming len(self._eqIndexTable_) is faster than len(self)
            if(self._eqIndexTable_[i] == eqIndex):
                return(i)
        raise(IndexError("phantomList index "+str(eqIndex)+" not found (write to it first)"))#if the function gets to here, no item with that index was found
    
    def __getitem__(self, eqIndex):
        # return(super(self.__class__, self).__getitem__(eqIndex)) #original functionality
        # if(type(eqIndex) is slice): safety check
        #     print("invalid use of phantomList")
        return(super(self.__class__, self).__getitem__(self._findLocalIndex_(eqIndex)))
    
    def __setitem__(self, eqIndex, value):
        # return(super(self.__class__, self).__setitem__(eqIndex, value)) #original functionality
        try:
            localIndex = self._findLocalIndex_(eqIndex)
            super(self.__class__, self).__setitem__(localIndex, value)
        except IndexError:
            self._eqIndexTable_.append(eqIndex)
            super(self.__class__, self).append(value)
    
    def __delitem__(self, eqIndex):
        localIndex = self._findLocalIndex_(eqIndex)
        self._eqIndexTable_.__delitem__(localIndex)
        super(self.__class__, self).__delitem__(localIndex)
    
    def clear(self):
        self._eqIndexTable_.clear()
        super(self.__class__, self).clear()
    
    def reverse(self):
        self._eqIndexTable_.reverse()
        super(self.__class__, self).reverse()
    
    def pop(self, eqIndex): #pop() returns the object, __delitem__() does not.
        localIndex = self._findLocalIndex_(eqIndex)
        self._eqIndexTable_.__delitem__(localIndex)
        return(super(self.__class__, self).pop(localIndex))
    
    def index(self, value):
        #print("WARNING!, phantomLists aren't really designed to use index()")
        localIndex = super(self.__class__, self).index(value)
        if(localIndex < 0):
            #print("WARNING!, phantomLists index() couldn't find", value)
            return(-1)
        return(self._eqIndexTable_[localIndex])
    
    # def extend(self, *args): #extend is allowed for unpickling, but shouldn't be used 
    #     print("ERROR!, phantomLists can't extend() (right now)")
    
    def remove(self, *args):
        print("ERROR!, phantomLists can't remove() (right now)")
    def sort(self, *args):
        print("ERROR!, phantomLists can't sort() (right now)")
    def insert(self, *args):
        print("ERROR!, phantomLists can't insert()") #because inserting changes the indices of all higher entries
    def append(self, *args):
        print("ERROR!, phantomLists can't append()") #because you need to provide an eqIndex as well as an object
# class phantomList(list):
#     """(sublist version) a list where you can set (list[index] = value) without the index existing, and read any (previously set) indices (even if they exceed len())"""
#     def __init__(self, initList=None, eqIndexTable=None):
#         if(type(initList) is phantomList):
#             copyList = list()
#             for i in range(len(initList)):
#                 item = initList._getitemraw_(i)
#                 copyList.append([item[0], item[1]]) #a slightly deep copy
#             list.__init__(self, copyList)
#         elif((initList is not None) and (eqIndexTable is not None)):
#             if(hasattr(initList, '__iter__') and hasattr(initList, '__len__') and \
#                 hasattr(eqIndexTable, '__iter__') and hasattr(eqIndexTable, '__len__')): #check if initList and eqIndexTable are itterable (list/tuple/etc.)
#                 if(len(initList) == len(eqIndexTable)):
#                     list.__init__(self, [[eqIndexTable[i], initList[i]] for i in range(len(initList))])
#                 else:
#                     raise(ValueError("phantomList init list & eqIndexTable length mismatch: "+str(len(initList))+" != "+str(len(eqIndexTable))))
#             else:
#                 raise(TypeError("phantomList init list/eqIndexTable invalid types: "+str(type(initList))+" and "+str(type(eqIndexTable))))
#         else:
#             if(initList is not None):
#                 print("can't init phantomList() with:", type(initList), initList)
#                 #raise TypeError
#             list.__init__(self)
    
#     def __repr__(self):
#         #return("phantomList"+super(self.__class__, self).__repr__())   #print list WITH eqIndex visible
#         tempList = [item for item in self] #uses iter()
#         return("phantomList"+tempList.__repr__())                       #print list WITHOUT eqIndex visible (like normal list)
    
#     def _findLocalIndex_(self, eqIndex):
#         for i in range(len(self)): #i'm assuming len(self._eqIndexTable_) is faster than len(self)
#             if(self._getitemraw_(i)[0] == eqIndex):
#                 return(i)
#         raise(IndexError("phantomList index "+str(eqIndex)+" not found (write to it first)"))#if the function gets to here, no item with that index was found
    
#     @property
#     def _eqIndexTable_(self): #(use with caution) generates a table of indices
#         return([self._getitemraw_(i)[0] for i in range(len(self))])
    
#     def _appendraw_(self, value): #note: 'value' needs to be a list where the [0] is the eqIndex and [1] is the stored value/object
#         super(self.__class__, self).append(value)
    
#     def _getitemraw_(self, localIndex): #
#         return(super(self.__class__, self).__getitem__(localIndex))
    
#     def _setitemraw_(self, localIndex, value): #note: 'value' needs to be a list where the [0] is the eqIndex and [1] is the stored value/object
#         super(self.__class__, self).__setitem__(localIndex, value)
    
#     def __getitem__(self, eqIndex): #note: slice() objects are not allowed
#         # return(super(self.__class__, self).__getitem__(eqIndex)) #original functionality
#         # if(type(eqIndex) is slice): safety check
#         #     print("invalid use of phantomList")
#         return(self._getitemraw_(self._findLocalIndex_(eqIndex))[1])
    
#     def __setitem__(self, eqIndex, value):
#         # return(super(self.__class__, self).__setitem__(eqIndex, value)) #original functionality
#         try:
#             localIndex = self._findLocalIndex_(eqIndex)
#             self._setitemraw_(localIndex, [eqIndex, value])
#         except:
#             self._appendraw_([eqIndex, value])
    
#     def __delitem__(self, eqIndex):
#         super(self.__class__, self).__delitem__(self._findLocalIndex_(eqIndex))
    
#     def pop(self, eqIndex): #pop() returns the object, __delitem__() does not.
#         return(super(self.__class__, self).pop(self._findLocalIndex_(eqIndex)))
    
#     def __iter__(self):
#         self.iterIndex = 0
#         return(self)
    
#     def __next__(self):
#         self.iterIndex += 1
#         if(self.iterIndex > len(self)):
#             raise StopIteration
#         return(super(self.__class__, self).__getitem__(self.iterIndex-1)[1])
    
#     def copyRaw(self):
#         #return(super(self.__class__, self).copy()) # a very shallow copy, as all (raw) entries of a phantomList are lists themselves (and therefore pointers)
#         copyList = phantomList()
#         for i in range(len(self)):
#             item = self._getitemraw_(i)
#             copyList._appendraw_([item[0], item[1]]) #a slightly deep copy
#         return(copyList)
    
#     def extend(self, anotherList):
#         if(type(anotherList) is phantomList):
#             for i in range(len(anotherList)):
#                 #self._appendraw_(anotherList._getitemraw_(i)) #a very shallow copy
#                 item = anotherList._getitemraw_(i)
#                 self._appendraw_([item[0], item[1]]) #a slightly deep copy
#         else:
#             print("ERROR!, phantomLists can only be extended with other phantomLists, not", type(anotherList))
    
#     def index(self, value):
#         #print("WARNING!, phantomLists aren't meant to use index()")
#         tempList = [item for item in self]
#         localIndex = tempList.index(value)
#         if(localIndex < 0):
#             #print("WARNING!, phantomLists index() couldn't find", value)
#             return(-1)
#         return(self._getitemraw_(localIndex)[0])
    
#     def remove(self, *args):
#         print("ERROR!, phantomLists can't remove() (right now)")
#     def sort(self, *args):
#         print("ERROR!, phantomLists can't sort() (right now)")
#     def copy(self):
#         print("ERROR!, phantomLists aren't meant to be copied, use .copyRaw() if target is another phantomList")
#         #return([self._getitemraw_(i) for i in range(len(self))])
#     def insert(self, *args):
#         print("ERROR!, phantomLists can't insert()") #because inserting changes the indices of all higher entries
#     def append(self, *args):
#         print("ERROR!, phantomLists can't append()") #because you need to provide an eqIndex as well as an object

class phantomObj: #an empty object, to add stuff to. I wanted to use the python builtin 'object()', but it seems to be readOnly (i can't add attributes)
    pass

class phantomDelete: #an (empty) object that indicates that a thing needs to be removed
    pass

class phantomClass:
    def __init__(self):
        #self._anyChange_ = False #was be removed because you can just check itemLog and deleteLog
        self._itemLog_ = []   #a list of pathTuples of what was added to the phantomClass
        self._deleteLog_ = [] #a list of pathTuples to be deleted
        self._customLog_ = [[],[],[]] #a list of custom instructions (which the user has to manually parse) each sublist is parsed at a different point (before/after itemLog/deleteLog)
    
    # @staticmethod
    # def _hasattr_(source, pathTuple): #just use _tryFetch_
    #     try:
    #         item = phantomClass._getAttrRecur_(source, pathTuple)
    #         return(True)
    #     except (AttributeError, IndexError):
    #         return(False)
    
    @staticmethod
    def _getIterItemRecur_(source, listPathTuple, deleteItem=False):
        if(len(listPathTuple) == 1): #if this is the last stop
            if(deleteItem):
                source.__delitem__(listPathTuple[0])
            else:
                return(source[listPathTuple[0]])
        else:
            return(phantomClass._getIterItemRecur_(source[listPathTuple[0]], listPathTuple[1:], deleteItem))
    
    @staticmethod
    def _getattr_(source, pathEntry, deleteItem=False): #an alternate version of __getattr__ which uses the pathEntry (entry of pathTuple) format
        if(type(pathEntry) is tuple): #if attribute is itterable
            return(phantomClass._getIterItemRecur_(getattr(source, pathEntry[0]), pathEntry[1:], deleteItem))
        else:
            if(deleteItem):
                delattr(source, pathEntry)
            else:
                return(getattr(source, pathEntry))
    
    @staticmethod
    def _getAttrRecur_(source, pathTuple, deleteItem=False):
        if(len(pathTuple) == 1): #if this is the last stop
            return(phantomClass._getattr_(source, pathTuple[0], deleteItem))
        else:
            return(phantomClass._getAttrRecur_(phantomClass._getattr_(source, pathTuple[0], False), pathTuple[1:], deleteItem))
    
    @staticmethod
    def _setIterItemRecur_(source, listPathTuple, value, appendOnIndexExcept=False):
        #self._anyChange_ = True  #already done in put()
        if(len(listPathTuple) == 1): #if this is the last stop
            try:
                source[listPathTuple[0]] = value
            except IndexError as excep:
                if(appendOnIndexExcept):
                    source.append(value)
                else:
                    raise(excep)
        else:
            nextSource = None #init var
            try:
                nextSource = source[listPathTuple[0]]
            except IndexError as excep: #for phantomLists, the error just means that entry hasnt been made yet
                if(type(source) is phantomList):
                    source[listPathTuple[0]] = phantomList()
                    nextSource = source[listPathTuple[0]]
                else:
                    # if(appendOnIndexExcept):
                    #     print("it seems you've run into an impossible append scenario.", listPathTuple, value)
                    raise(excep) #in this case, it is (near?) impossible to KNOW what to append
            phantomClass._setIterItemRecur_(nextSource, listPathTuple[1:], value, appendOnIndexExcept)
    
    @staticmethod
    def _setattr_(source, pathEntry, value, raiseError=False, appendOnIndexExcept=False): #an alternate version of __setattr__ which uses the pathEntry (entry of pathTuple) format
        #self._anyChange_ = True  #already done in put()
        if(type(pathEntry) is tuple): #if attribute is itterable
            if(not hasattr(source, pathEntry[0])):
                if(raiseError):
                    raise(AttributeError("hasattr(itterable) error, "+str(pathEntry)))
                setattr(source, pathEntry[0], phantomList())
            phantomClass._setIterItemRecur_(getattr(source, pathEntry[0]), pathEntry[1:], value, appendOnIndexExcept)
        else:
            if((not hasattr(source, pathEntry)) if raiseError else False):
                raise(AttributeError("hasattr(attribute) error, "+str(pathEntry)))
            setattr(source, pathEntry, value)
    
    @staticmethod
    def _setAttrRecur_(source, pathTuple, value, raiseError=False, appendOnIndexExcept=False):
        #self._anyChange_ = True  #already done in put()
        if(len(pathTuple) == 1): #if this is the last stop
            phantomClass._setattr_(source, pathTuple[0], value, raiseError, appendOnIndexExcept)
        else:
            #nextSource = None #init var
            try:
                nextSource = phantomClass._getattr_(source, pathTuple[0])
            except Exception as excep:
                if(raiseError): #if the source is the original object (not recommended)
                    raise(excep)
                nextObj = phantomObj() #create an arbitrary (empty) object to put stuff in
                if(type(pathTuple[1]) is tuple): #if the next object is actually a list
                    nextObj = phantomList()
                phantomClass._setattr_(source, pathTuple[0], nextObj, raiseError, appendOnIndexExcept) #create an object/list to put the subObjects in (note: raiseError will always be False)
                nextSource = phantomClass._getattr_(source, pathTuple[0]) #a somewhat slow (but safe) approach
                #nextSource = nextObj  #a quicker and dirtier (pointer) approach
            phantomClass._setAttrRecur_(nextSource, pathTuple[1:], value, raiseError, appendOnIndexExcept) #note: nextSource is equivalent to 'phantomClass._getattr(source, pathTuple[0])' but faster
    
    @staticmethod
    def _tryFetch_(source, pathTuple):
        """attempt to fetch an attribute/entry
            if the path does not exist, this will return(False, None)"""
        try:
            item = phantomClass._getAttrRecur_(source, pathTuple)
            return(True, item)
        except (AttributeError, IndexError):
            return(False, None)
    
    # def _tryLogFetch_(self, pathTuple): #(untested!) should work, but is pretty silly
    #     #compare all entries of _itemLog_ to pathTuple to see if it exists
    #     for itemLogIndex in range(len(self._itemLog_)):
    #         existingPathTuple = self._itemLog_[itemLogIndex]
    #         if(len(pathTuple) >= len(existingPathTuple)): #if pathTuple is longer than existingPath, 
    #             perfectMatch = True #init var
    #             for i in range(len(existingPathTuple)):
    #                 if(not perfectMatch): #little speed upgrade
    #                     continue
    #                 if(i < (len(existingPathTuple)-1)):
    #                     if(type(pathTuple[i]) is not type(existingPathTuple[i])):
    #                         perfectMatch = False
    #                         continue
    #                     if(type(pathTuple[i]) is tuple): #if attribute is itterable
    #                         for j in range(len(pathTuple[i])):
    #                             if(not perfectMatch): #little speed upgrade
    #                                 continue
    #                             if(pathTuple[i][j] != existingPathTuple[i][j]):
    #                                 perfectMatch = False
    #                                 continue
    #                     elif(pathTuple[i] != existingPathTuple[i]):
    #                         perfectMatch = False
    #                 else:
    #                     if(type(pathTuple[i]) is tuple): #if attribute is itterable
    #                         if(type(existingPathTuple[i]) is tuple): #if attribute is itterable
    #                             for j in range(len(pathTuple[i])):
    #                                 if(not perfectMatch): #little speed upgrade
    #                                     continue
    #                                 if(pathTuple[i][j] != existingPathTuple[i][j]):
    #                                     perfectMatch = False
    #                                     continue
    #                         else: #if pathTuple[i] wants entry of an itterable, and existingPathTuple[i] has the whole itterable
    #                             if(pathTuple[i][0] != existingPathTuple[i]): #as long as the attribute names match
    #                                 perfectMatch = False
    #                     elif(type(existingPathTuple[i]) is tuple): #if pathTuple wants the whole itterable, but existingPathTuple only has a specific entry
    #                         perfectMatch = False
    #                     elif(pathTuple[i] != existingPathTuple[i]):
    #                         perfectMatch = False
    #             if(perfectMatch):
    #                 return(phantomClass._tryFetch_(self, pathTuple))
    #     return(False, None)
    
    @staticmethod
    def _delete_(source, pathTuple):
        """delete an attribute/entry (not the same as logDelete())"""
        phantomClass._getAttrRecur_(source, pathTuple, True)
    
    def _put_(self, pathTuple, value):
        """set an attribute/entry
            if the path to the item does not exist, it will be made"""
        phantomClass._setAttrRecur_(self, pathTuple, value)
        #self._anyChange_ = True
        self._itemLog_.append(pathTuple)
    
    def _append_(self, unfinishedpathTuple, appendedList):
        """a shortcut for appending (single!) items from a list that was recently appended to"""
        lastPathTupleEntry = [item for item in unfinishedpathTuple[-1]]
        lastPathTupleEntry.append(len(appendedList)-1)
        finishedPathTuple = tuple([(unfinishedpathTuple[i] if (i<(len(unfinishedpathTuple)-1)) else tuple(lastPathTupleEntry)) for i in range(len(unfinishedpathTuple))])
        self._put_(finishedPathTuple, appendedList[-1])
    
    def __setitem__(self, pathTuple, value): #just a fun proxy for the put() function
        """(proxy for put()) set an attribute/entry
            if the path to the item does not exist, it will be made"""
        self._put_(pathTuple, value)
    
    def _logDelete_(self, pathTuple): #TBD, figure out an altername way to do this maybe?)
        """log attribute/entry for deletion"""
        self._deleteLog_.append(pathTuple)
    
    def _custom_(self, customInstruction, priority=0):
        """log custom instruction (to be parsed by custom user function).
            priority ranges from 0 to 2, and determines at which point of the merger the instruction will be executed (before/after merging/deleting)"""
        self._customLog_[priority].append(customInstruction)
    
    @property
    def _hasData_(self): #i'm not 100% satisfied with this name, '_anyData_' or '_anyChange_' could also work
        """returns the number of additions/alterations/deletions/customInstructions logged"""
        return(len(self._itemLog_)+len(self._deleteLog_)+(len(self._customLog_[0])+len(self._customLog_[1])+len(self._customLog_[2])))
    
    # def __getitem__(self, pathTuple): #old code, no longer makes sense
    #     print("__getitem__", pathTuple)
    #     # see if 'self' has the item, and if not read it from original
    #     # if 'self' has the object, but the object is incomplete (???), then read (copy) from original and apply changemap changes to it
    #     # also check added objects
    #     try:
    #         itemToReturn = self. #see if 'self' has it
    #     except:
    #         print("ass")
    #     return(

class phantomClassSimple: #for the most part this is just better
    def __init__(self):
        #self._anyChange_ = False #was be removed because you can just check itemLog and deleteLog
        self._items_ = []     #a list of item (this is the counterpart to _itemLog_)
        self._itemLog_ = []   #a list of pathTuples to be altered/added
        self._deleteLog_ = [] #a list of pathTuples to be deleted
        self._customLog_ = [[],[],[]] #a list of custom instructions (which the user has to manually parse) each sublist is parsed at a different point (before/after itemLog/deleteLog)
    
    # def _tryLogFetch_(self, pathTuple): #(untested!) should work, but is pretty silly
    #     #compare all entries of _itemLog_ to pathTuple to see if it exists
    #     for itemLogIndex in range(len(self._itemLog_)):
    #         existingPathTuple = self._itemLog_[itemLogIndex]
    #         if(len(pathTuple) >= len(existingPathTuple)): #if pathTuple is longer than existingPath, 
    #             perfectMatch = True #init var
    #             for i in range(len(existingPathTuple)):
    #                 if(not perfectMatch): #little speed upgrade
    #                     continue
    #                 if(i < (len(existingPathTuple)-1)):
    #                     if(type(pathTuple[i]) is not type(existingPathTuple[i])):
    #                         perfectMatch = False
    #                         continue
    #                     if(type(pathTuple[i]) is tuple): #if attribute is itterable
    #                         for j in range(len(pathTuple[i])):
    #                             if(not perfectMatch): #little speed upgrade
    #                                 continue
    #                             if(pathTuple[i][j] != existingPathTuple[i][j]):
    #                                 perfectMatch = False
    #                                 continue
    #                     elif(pathTuple[i] != existingPathTuple[i]):
    #                         perfectMatch = False
    #                 else:
    #                     if(type(pathTuple[i]) is tuple): #if attribute is itterable
    #                         if(type(existingPathTuple[i]) is tuple): #if attribute is itterable
    #                             for j in range(len(pathTuple[i])):
    #                                 if(not perfectMatch): #little speed upgrade
    #                                     continue
    #                                 if(pathTuple[i][j] != existingPathTuple[i][j]):
    #                                     perfectMatch = False
    #                                     continue
    #                         else: #if pathTuple[i] wants entry of an itterable, and existingPathTuple[i] has the whole itterable
    #                             if(pathTuple[i][0] != existingPathTuple[i]): #as long as the attribute names match
    #                                 perfectMatch = False
    #                     elif(type(existingPathTuple[i]) is tuple): #if pathTuple wants the whole itterable, but existingPathTuple only has a specific entry
    #                         perfectMatch = False
    #                     elif(pathTuple[i] != existingPathTuple[i]):
    #                         perfectMatch = False
    #             if(perfectMatch):
    #                 if(len(pathTuple) > len(existingPathTuple)): #if pathTuple is a subset of existingPathTuple
    #                     return(phantomClass._tryFetch_(self._items_[itemLogIndex], pathTuple[len(existingPathTuple):]))
    #                 elif((type(pathTuple[-1]) is tuple) and (type(existingPathTuple[-1]) is not tuple)): #if pathTuple[-1] wants entry of an itterable, and existingPathTuple[-1] has the whole itterable
    #                     try:
    #                         return(True, phantomClass._getIterItemRecur_(self._items_[itemLogIndex], pathTuple[-1][1:], False)) #attemp to fetch pathTuple[-1][1:] entry from stored itterable
    #                     except:
    #                         return(False, None)
    #                 else: #if pathTuple perfectly matches existingPathTuple
    #                     return(True, self._items_[itemLogIndex])
    #     return(False, None)
    
    def _put_(self, pathTuple, value):
        """log attribute/entry for creation/alteration"""
        self._itemLog_.append(pathTuple)
        self._items_.append(value)
    
    def _append_(self, unfinishedpathTuple, appendedList):
        """a shortcut for appending (single!) items from a list that was recently appended to"""
        lastPathTupleEntry = [item for item in unfinishedpathTuple[-1]]
        lastPathTupleEntry.append(len(appendedList)-1)
        finishedPathTuple = tuple([(appendedList[i] if (i<(len(unfinishedpathTuple)-1)) else tuple(lastPathTupleEntry)) for i in range(len(unfinishedpathTuple))])
        self._put_(finishedPathTuple, appendedList[-1])
    
    def __setitem__(self, pathTuple, value): #just a fun proxy for the put() function
        """(proxy for put()) set an attribute/entry
            (this class only uses _itemLog_[])"""
        self._put_(pathTuple, value)
    
    def _logDelete_(self, pathTuple):
        """log attribute/entry for deletion"""
        self._deleteLog_.append(pathTuple)
    
    def _custom_(self, customInstruction, priority=0):
        """log custom instruction (to be parsed by custom user function).
            priority ranges from 0 to 2, and determines at which point of the merger the instruction will be executed (before/after merging/deleting)"""
        self._customLog_[priority].append(customInstruction)
    
    @property
    def _hasData_(self): #i'm not 100% satisfied with this name, '_anyData_' or '_anyChange_' could also work
        """returns the number of additions/alterations/deletions/customInstructions logged"""
        return(len(self._itemLog_)+len(self._deleteLog_)+(len(self._customLog_[0])+len(self._customLog_[1])+len(self._customLog_[2])))

def mergePhantom(original, phantom, customHandler=None):
    """overwrite original data with phantom data (using phantom._itemlog_)"""
    if((customHandler) and callable(customHandler)):
        for customInstruction in phantom._customLog_[0]:
            customHandler(original, customInstruction)
    elif(len(phantom._customLog_[0])>0):
        print("can't parse", len(phantom._customLog_[0]), "custom phantom instructions!(0)")
    
    for pathTuple in phantom._itemLog_:
        valueExists = True;  value = None #init vars
        if((type(phantom) is phantomClassSimple) or (hasattr(phantom, '_items_'))):
            value = phantom._items_[phantom._itemLog_.index(pathTuple)] #retrieve item from _items_ at same index
        else:
            valueExists, value = phantom._tryFetch_(phantom, pathTuple)
        if(not valueExists):
            print("ERROR!, _itemlog_ has a path that can't be fetched:", pathTuple)
            continue #skip this one to avoid errors
        print("merging", pathTuple, value)
        try:
            phantomClass._setAttrRecur_(original, pathTuple, value, True, True) #use appending right from the start
        #     phantomClass._setAttrRecur_(original, pathTuple, value, True, False) #first try without appending
        # except IndexError: #if an list index exists in the phantom, but not in the original, it may have been appended
        #     print("warning! appending item because (phantom) index was out of range")
        #     phantomClass._setAttrRecur_(original, pathTuple, value, True, True)
        except Exception as excep:
            print("merging excep:", excep)
            raise(excep) #i dont expect to have exceptions, so if there are any, you should raise them
    
    if((customHandler) and callable(customHandler)):
        for customInstruction in phantom._customLog_[1]:
            customHandler(original, customInstruction)
    elif(len(phantom._customLog_[1])>0):
        print("can't parse", len(phantom._customLog_[1]), "custom phantom instructions!(1)")
    
    ## merge _deleteLog_ after _itemLog_, because _deleteLog_ may change array indices
    for pathTuple in phantom._deleteLog_:
        print("deleting", pathTuple)
        valueExists, value = phantom._tryFetch_(original, pathTuple)
        print(valueExists, value)
        phantomClass._delete_(original, pathTuple)
        valueExists, value = phantom._tryFetch_(original, pathTuple)
        print(valueExists, value)
    
    if((customHandler) and callable(customHandler)):
        for customInstruction in phantom._customLog_[2]:
            customHandler(original, customInstruction)
    elif(len(phantom._customLog_[2])>0):
        print("can't parse", len(phantom._customLog_[2]), "custom phantom instructions!(2)")
        

# import builtins
# def isBuiltin(obj):
#     return(type(obj).__name__ in dir(builtins))

# def simpleDir(obj):
#     fullDir = dir(obj)
#     returnList = []
#     for attrName in fullDir:
#         if((not attrName.startswith('_')) and (not callable(getattr(obj, attrName)))):
#             returnList.append(attrName)
#     return(returnList)

# phantomTypes = (phantomList, phantomObj, phantomClass)

# def mergePhantomNoLog(original, phantom, customHandler=None):     #UNFINISHED, because using the _itemLog_ was just that much simpler
#     """overwrite original data with phantom data,
#         NOTE: (without an _itemlog_) this is done by (recursively) unpacking objects/lists 
#                 untill builtin types are reached (then copy those)""" #the problem with this approach is stuff like numpy arrays. You could solve this with a lookup table (or just _itemLog_)
#     for attrName in simpleDir(phantom):
#         if(hasattr(original, attrName)):
#             if(



if __name__ == "__main__":      #some little tests
    ## phantomList examples
    aList = phantomList()
    aList[123] = 'ass' #write any index you want
    aList[456] = 'also'
    print("aList", aList)
    print(aList.index('also'))
    bList = phantomList()
    bList.extend(aList)
    print("bList", bList)
    cList = phantomList(aList) #initializing with another phantomList
    print("cList", cList)
    dList = phantomList([item for item in aList], aList._eqIndexTable_) #initializing with different types, but the appropriate data
    print("dList", dList)
    print(dList._eqIndexTable_)

    ## phantomClass examples
    from Map import Map
    
    aMap = Map()
    
    phantomMap = phantomClass()
    
    alterPath = ('clockStart',)
    phantomMap._put_(alterPath, 123456789)
    #phantomMap[alterPath] = 123456789
    print(phantomMap._tryFetch_(phantomMap, alterPath))
    
    print(aMap.clockStart)
    mergePhantom(aMap, phantomMap)
    print(aMap.clockStart)
    print(); print()
    
    
    phantomMap = phantomClass()
    
    appendPath = (('left_cone_list', len(aMap.left_cone_list)),) #if appending, it does not matter how high the index is AS LONG AS it's higher than the max existing index in the list.
    phantomMap._put_(appendPath, Map.Cone())
    #phantomMap[appendPath] = Map.Cone()
    print(phantomMap._tryFetch_(phantomMap, appendPath))
    
    print(aMap.left_cone_list)
    mergePhantom(aMap, phantomMap)
    print(aMap.left_cone_list)
    print(); print()
    
    
    phantomMap = phantomClass()
    
    appendPath = (('left_cone_list', 0),)
    phantomMap._logDelete_(appendPath)
    
    print(aMap.left_cone_list)
    mergePhantom(aMap, phantomMap)
    print(aMap.left_cone_list)
    print(); print()