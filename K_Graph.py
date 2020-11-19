class K_Link:
    def __init__(self, from_node, to_node):
        self.From = from_node
        self.To = to_node
        self.Data = []


class K_Node:
    def __init__(self):
        self.ParentNode = None
        self.ChildrenNodes = []
        self.Level = 0
        self.TraversalFlag = 0
        self.Data = []
        self.Link = None

    def AppendChild(self, child_node): self.ChildrenNodes.append(child_node)

    def Link2Nodes(self, parent_node, child_node, linkage=0):
        if linkage == 0 : self.Link = K_Link(parent_node, child_node)
        else : self.Link = linkage
        self.ParentNode = parent_node

    def AppendData(self, arg):
        self.Data.append(arg)

    def GetLevel(self): return self.Level

    def SetLevel(self, n):
        self.Level = n

    def ExecuteMetod(self, method_name, t, args=0):
        if method_name == 'Print' : self.Print(t, args) ;
        if method_name == 'Plot' : self.Plot(t, args) ;
        if method_name == 'Forces and Moments Evaluation' : self.ForcesAndTorquesEvaluation(t, args)
        if method_name == 'Compute Degrees Of Freedom' :
            self.ComputeDOFs()
            if self.DOFs.__len__() > 0 : return self
            return 0
        if method_name == 'Add Time Step' : self.AddTimeStep(t)
        if method_name == 'Update Linkage Moment And Torques' :
            if self.Link != None : self.Link.UpdateLinkageForcesAndTorques()

    def ForcesAndTorquesEvaluation(self,t, arg) :
        return

    def Print(self, t, arg):
        if self.ParentNode == None :
            print (self.Data, 'Root Node')
        else :
            print (self.Data, '(',self.ParentNode.Data,')')

    def Plot(self, t, arg):
        return


class K_Tree_Level_Information:
    def __init__(self, level, starter_node):
        self.LevelDepth = level
        self.NodesList = [starter_node]

    def AppendNode(self, node):
        self.NodesList.append(node)


class K_Tree:
    def __init__(self, root_node) :
        self.Depth=1
        self.RootNode = root_node
        self.LevelInformation = [K_Tree_Level_Information(self.Depth, self.RootNode)]
        self.Data= []

    def GetRootNode(self): return(self.RootNode)

    def InsertLeaf(self, node, parent_node, link):

        parent_node.AppendChild(node)                     # Append node to the parent children note list
        node.SetLevel(parent_node.Level+1)                # Set the node level

        if self.Depth == parent_node.Level+1 :
            self.Depth +=1  # The node we are trying to add creates a new level in the tree
            self.LevelInformation.append(K_Tree_Level_Information(self.Depth, node))
        else :
            self.LevelInformation[node.Level].AppendNode(node)
            #self.LevelInformation[self.Depth-1].AppendNode(node) # Append the new node to the level list

        node.Link2Nodes(parent_node,node, link)

    def TraverseFromRoottoLeaves(self, func_to_execute_during_traversal, t, args):
        for current_level in self.LevelInformation :
                print ('----------------------------------------------------------------------------------------------')
                for current_node in current_level.NodesList :
                    current_node.ExecuteMetod(func_to_execute_during_traversal, t, args)
                print ('\n')

    def TraverseFromLeavestoRoot(self, func_to_execute_during_traversal, t=-1, args=0):
        self.Data = []
        for current_level in reversed(self.LevelInformation) :
                print ('----------------------------------------------------------------------------------------------')
                for current_node in current_level.NodesList :
                    data = current_node.ExecuteMetod(func_to_execute_during_traversal, t, args)
                    if data != 0 :
                        self.Data.append(data)
                print ('\n')

    def ExecuteMetod(self,  method_name, t, args=0):
        if method_name == 'Print' : self.Print(args[0]) ;
        if method_name == 'Plot' : self.Plot(t, args[0], args[1]) ;
        if method_name == 'Forces and Moments Evaluation' : self.ForcesAndTorquesEvaluation(args) ;
        if method_name == 'Get Description' : return args + self.GetDescription(args)
        if method_name == 'Update Linkage Moment And Torques' : self.ParentNode.UpdateLinkageForcesAndTorques()

    def Print(self, direction, t, args):
        if direction == "Top to Bottom" :
            self.TraverseFromRoottoLeaves('Print', t, args)
        elif direction == "Bottom to Top":
            self.TraverseFromLeavestoRoot('Print', t, args)
        else :
            print ('Print Direction unknown')
