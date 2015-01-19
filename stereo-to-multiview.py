import nuke
import nukescripts
import os
import re
import glob
import errno

'''
ttmEngine

this class operates in Nuke 6.1v3, it has worker methods to create, execute, and render nodes necessary for
the conversion of two view to multi view source clips.

Prior to usage of methods, create your project folder, and inside it, a subfolder named 'source'.

Place the left & right clips or frame sequences in the newly created source folder.

After this, paste this script into the script editor of Nuke and you will have a interactive panel
'''


'''
ttmEngineController is the delegate for message passing between engine and view

ttmEngineView is the panel interface

ttmEngine does the actual automation
    - tracking
    - solving
    - rendering

'''

class ttmEngineController:

    def __init__( self ):
        self.view = ttmEngineView()

        # Bootup protocol
        self.bootUp()

        # Show panel
        self.view.show()


    def bootUp( self ):

        # Disable everything except init knobs
        self.view.enableLoadKnob( False )
        self.view.enableTrackKnobs( False )
        self.view.enableSolveKnobs( False )
        self.view.enableRenderKnobs( False )

        # Do bootup checking

        # Check if there is left, right, & joined source
        leftSource = nuke.toNode( 'LEFT_SOURCE' )
        rightSource = nuke.toNode( 'RIGHT_SOURCE' )
        joinedSource = nuke.toNode( 'JOINED_SOURCE' )

        if (not leftSource == None) and (not rightSource == None) and (not joinedSource == None):
            # Check if project paths match
            leftSourcePath = os.path.split(os.path.split(leftSource['file'].getValue())[0])[0]
            rightSourcePath = os.path.split(os.path.split(rightSource['file'].getValue())[0])[0]

            if ( leftSourcePath == rightSourcePath ):
                # They are the same, we can init engine
                leftSourceFile = os.path.split(leftSource['file'].getValue())[1]
                rightSourceFile = os.path.split(rightSource['file'].getValue())[1]
                self.initEngine(leftSourcePath, leftSourceFile, rightSourceFile)

                self.view.enableInitKnobs( False )
                # Enable load, track, render
                self.view.enableLoadKnob( True )
                self.view.enableTrackKnobs( True )
                self.view.enableRenderKnobs( True )

                # Check for existence of camera tracker & camera
                cameraTracker = nuke.toNode('CAMERA_TRACKER')

                if ( not cameraTracker == None ):
                    # Found a camera tracker, now enable solve knobs
                    self.view.enableSolveKnobs(True)


    def initEngine( self, projectPath, leftSourceFileName, rightSourceFileName ):

        # disable init knobs
        self.view.enableInitKnobs( False )

        # load engine
        self.engine = ttmEngine( projectPath, leftSourceFileName, rightSourceFileName )
        self.view.enableLoadKnob( True )

    def load( self ):

        # Load sources
        self.engine.load()

        # Enable track & render knobs
        self.view.enableTrackKnobs( True )
        self.view.enableRenderKnobs( True )

    def render( self, useCamera, outputInterm, frameRange, render_a_enabled, render_b_enabled, render_a_dim, render_b_dim ):

        # Append dimensions into a list
        dimensions = list()

        if ( not render_a_enabled ):
            render_a_dim = None
        else:
            dimensions.append( (int(render_a_dim[0]), int(render_a_dim[1])) )

        if ( not render_b_enabled ):
            render_b_dim = None
        else:
            dimensions.append( (int(render_b_dim[0]), int(render_b_dim[1])) )

        # Sort frame ranges
        #make it  into int
        frameRange = (int(frameRange[0]), int(frameRange[1]))

        if (frameRange[0] == -1 and frameRange[0] == -1):
            frameRange[0] == nuke.root()['first_frame'].getValue()
            frameRange[1] == nuke.root()['last_frame'].getValue()

        if ( frameRange[0] > frameRange[1] ):
            print( 'Error: End frame greater than start frame' )
            return

        self.engine.render( useCamera, outputInterm, frameRange, dimensions )

class ttmEngineView(nukescripts.PythonPanel):

    def __init__( self ):

        # Panel Title
        nukescripts.PythonPanel.__init__( self, 'ttmEngine User Interface' );

        # Project preset knobs
        self.knobs = dict()

        self.knobs['init_path'] = nuke.String_Knob('init_path', 'Project Path:', 'C:/MyProject')
        self.knobs['init_lsource'] = nuke.String_Knob('init_lsource', 'Left Source Filename:', 'left.avi')
        self.knobs['init_rsource'] = nuke.String_Knob('init_rsource', 'Right Source Filename:', 'right.avi')

        # Init Engine Pyscript Knob
        self.knobs['init_execute'] = nuke.PyScript_Knob('init_execute', 'Initiate Engine',
                                                        '''_ttmGlobal.initEngine( _ttmGlobal.view.knobs['init_path'].value(),
                                                        _ttmGlobal.view.knobs['init_lsource'].value(),  _ttmGlobal.view.knobs['init_rsource'].value())''')

        # Load Source Pyscript knob
        self.knobs['load_execute'] = nuke.PyScript_Knob('load_execute', 'Load Source',
                                                 '_ttmGlobal.load()')
        # Track Source Pyscript knobs
        self.knobs['track_startFrame'] = nuke.Int_Knob('track_startFrame', 'Track Start Frame:', 0)
        self.knobs['track_endFrame'] = nuke.Int_Knob('track_endFrame', 'Track End Frame:', 0)
        self.knobs['track_increment'] = nuke.Int_Knob('track_increment', 'Track Frame Increment:', 500)
        self.knobs['track_execute'] = nuke.PyScript_Knob('track_execute', 'Track Source',
                                                  '''_ttmGlobal.track( _ttmGlobal.view.knobs['track_startFrame'].value(),
                                                  _ttmGlobal.view.knobs['track_endFrame'].value(), _ttmGlobal.view.knobs['track_increment'].value())''')

        # Solve Track data Pyscript knob

        # Render Knobs
        self.knobs['render_useCam'] = nuke.Boolean_Knob( 'render_useCam', 'Use Camera for Solver', False )
        self.knobs['render_interm'] = nuke.Boolean_Knob( 'render_interm', 'Render 640x400 8 View Format', False )

        self.knobs['render_startFrame'] = nuke.Int_Knob('render_startFrame', 'Render Start Frame:', -1)
        self.knobs['render_endFrame'] = nuke.Int_Knob('render_endFrame', 'Render End Frame:', -1)

        # a
        self.knobs['render_a_enabled'] = nuke.Boolean_Knob( 'render_a_enabled', 'Render', False )
        self.knobs['render_a_dim'] = nuke.XY_Knob('render_a_dim', 'Output Combined Format A')
        # b
        self.knobs['render_b_enabled'] = nuke.Boolean_Knob( 'render_b_enabled', 'Render', False )
        self.knobs['render_b_dim'] = nuke.XY_Knob( 'render_b_dim', 'Output Combined Format B')
        # button
        self.knobs['render_execute'] = nuke.PyScript_Knob('render_execute', 'Start Render',
                                                          '''_ttmGlobal.render( _ttmGlobal.view.knobs['render_useCam'].value(),
                                                          _ttmGlobal.view.knobs['render_interm'].value(),
                                                          (_ttmGlobal.view.knobs['render_startFrame'].value(), _ttmGlobal.view.knobs['render_endFrame'].value()),
                                                          _ttmGlobal.view.knobs['render_a_enabled'].value(), _ttmGlobal.view.knobs['render_b_enabled'].value(),
                                                          (_ttmGlobal.view.knobs['render_a_dim'].x(), _ttmGlobal.view.knobs['render_a_dim'].y()),
                                                          (_ttmGlobal.view.knobs['render_b_dim'].x(), _ttmGlobal.view.knobs['render_b_dim'].y()))''')

        # add the knobs
        self.__redrawKnobs()

    def __redrawKnobs( self ):
        # remove all visible knobs
        for k in self.knobs:
            try:
                self.removeKnob(self.knobs[k])
            except ValueError:
                print( 'error: Unable to remove nonexistent knob' )

        # Add all knobs to pane

        # Init
        self.addKnob(nuke.Text_Knob( 'init_title', 'Engine Setup' ))
        self.addKnob(self.knobs['init_path'])
        self.knobs['init_path'].setFlag(nuke.STARTLINE)
        self.addKnob(self.knobs['init_lsource'])
        self.knobs['init_lsource'].setFlag(nuke.STARTLINE)
        self.addKnob(self.knobs['init_rsource'])
        self.knobs['init_rsource'].setFlag(nuke.STARTLINE)
        self.addKnob(self.knobs['init_execute'])
        self.knobs['init_execute'].setFlag(nuke.STARTLINE)

        # Load
        self.addKnob( nuke.Text_Knob( 'load_title', 'Load' ))
        self.addKnob(self.knobs['load_execute'])
        self.knobs['load_execute'].setFlag(nuke.STARTLINE)

        # Track
        self.addKnob( nuke.Text_Knob( 'track_title', 'Track' ))
        self.addKnob(self.knobs['track_startFrame'])
        self.addKnob(self.knobs['track_endFrame'])
        self.knobs['track_endFrame'].setFlag(nuke.STARTLINE)
        self.addKnob(self.knobs['track_increment'])
        self.knobs['track_increment'].setFlag(nuke.STARTLINE)
        self.addKnob(self.knobs['track_execute'])
        self.knobs['track_execute'].setFlag(nuke.STARTLINE)

       # self.addKnob( nuke.Text_Knob( 'solve_title', 'Solve' )) NEED TO IMPLEMENT SOLVE FUNCTION

        # Render
        self.addKnob( nuke.Text_Knob( 'render_title', 'Render' ))
        self.addKnob(self.knobs['render_useCam'])
        self.knobs['render_useCam'].setFlag(nuke.STARTLINE)
        self.addKnob(self.knobs['render_interm'])
        self.knobs['render_interm'].setFlag(nuke.STARTLINE)

        self.addKnob(self.knobs['render_startFrame'])
        self.addKnob(self.knobs['render_endFrame'])
        self.knobs['render_endFrame'].setFlag(nuke.STARTLINE)

        self.addKnob(self.knobs['render_a_dim'])
        self.knobs['render_a_dim'].setFlag(nuke.STARTLINE)
        self.addKnob(self.knobs['render_a_enabled'])

        self.addKnob(self.knobs['render_b_dim'])
        self.knobs['render_b_dim'].setFlag(nuke.STARTLINE)
        self.addKnob(self.knobs['render_b_enabled'])

        self.addKnob(self.knobs['render_execute'])
        self.knobs['render_execute'].setFlag(nuke.STARTLINE)

    def enableInitKnobs( self, enabled ):
        for k in self.knobs:
            if ( not re.match( 'init', k) == None ):
                self.knobs[k].setEnabled( enabled )

    def enableLoadKnob( self, enabled ):
        for k in self.knobs:
            if (not re.match( 'load', k) == None ):
                self.knobs[k].setEnabled( enabled )

    def enableTrackKnobs( self, enabled ):
        for k in self.knobs:
            if ( not re.match( 'track', k) == None ):
                self.knobs[k].setEnabled( enabled )

    def enableSolveKnobs( self, enabled ):
        for k in self.knobs:
            if ( not re.match( 'solve', k) == None ):
                self.knobs[k].setEnabled( enabled )

    def enableRenderKnobs( self, enabled ):
        for k in self.knobs:
            if ( not re.match( 'render', k) == None ):
                self.knobs[k].setEnabled( enabled )

class Enum(set):
    def __getattr__( self, name ):
        if name in self:
            return name
        raise AttributeError

EngineStatus = Enum(['none', 'track', 'solve', 'render'])

class ttmEngine:

    # Following methods will belong to Standard Nuke Library in the near future
    def __makeDir(self, dirPath):

        # If directory does not exist, create it
        if not os.path.exists( dirPath ):
            try:
                # Try make directory
                os.makedirs( dirPath )
            except OSError, e:
                if e.errno != errno.EEXIST:
                    raise

    def __deleteNodesWithType( self, nodeTypeStr ):
        nodes = nuke.allNodes(nodeTypeStr)
        if len(nodes) > 0:
            for n in nodes:
                nuke.delete(n)


	def __setupMultiView( self, views=[ ('left',(0,1,0)), ('right',(1,0,0) ) ] ):

        '''
		Set up the nuke project with an arbitrary amount of colour coded views
		args:
        views  -  nested list with view names and rgb tuples for each view. rgb values are assumed to be normalise, eg red = (1,0,0)
		'''

		newViews = []
		for v in views:   # CYCLE THROUGH EACH REQUESTED VIEW
		    name = v[0]   # GRAB THE CURRENT VIEWS NAME
		    col = v[1]    # GRAB THE CURRENT VIEWS COLOUR
	        rgb = tuple( [ int(v*255) for v in col ] ) #CONVERT FLOAT TO 8BIT INT AND RETURN A TUPLE
		    hexCol = '#%02x%02x%02x' % rgb             #CONVERT INTEGER NUMBERS TO HEX CODE
		    curView = '%s %s' % ( name, hexCol )       #COMBINE NAME AND HEX COLOUR TO SCRIPT SYNTAX
		    newViews.append( curView )      # COLLECT ALL REQUESTED VIEWS

		 # COMBINE ALL VIEWS WITH LINE BREAK AND INITIALISE THE VIEWS KNOB WITH THE RESULTING SCRIPT SYNTAX
		nuke.root().knob('views').fromScript( '\n'.join( newViews ) )

    def __newRead( self, nodeName, filePath ):

        # Setup a proper read node with correct frame ranges & format
        readNode = nuke.nodes.Read(name=nodeName)
        readNode['file'].fromUserText(filePath)

        return readNode

    def __newCameraTracker( self, sourceNode, frameStart, frameEnd ):

        # Creates a CameraTracker node from source
        camTracker = nuke.nodes.CameraTracker1_0()

        # Set Custom Range
        camTracker['analysisRange'].setValue(1)
        camTracker['analysisStart'].setValue(frameStart)
        camTracker['analysisStop'].setValue(frameEnd)

        # Connect input to source
        camTracker.setInput(0,sourceNode)

        return camTracker

    # ttmEngine class begins here


	def __init__( self, rootDir, leftSource, rightSource ):

        # error strings
        self.errStr = 'ttmEngine error @ '
        self.warnStr = 'ttmEngine warning @ '

		# init properties

		self.projectName = os.path.split(rootDir)[1] # private project name is the directory name to make things simple

        self.paths = { 'root'       :       rootDir,
                       'source'     :       os.path.join( rootDir, 'source' ),
                       'l_source'   :       os.path.join( rootDir, 'source', leftSource ),
                       'r_source'   :       os.path.join( rootDir, 'source', rightSource ),
                       'track'      :       os.path.join( rootDir, 'track' ),
                       'solve'      :       os.path.join( rootDir, 'solve' ),
                       'render'     :       os.path.join( rootDir, 'render' ),
                       'output'     :       os.path.join( rootDir, 'output' ) }

        # Make directories
        self.__makeDir( os.path.join( self.paths['track'] ))
        self.__makeDir( os.path.join( self.paths['solve'] ))
        self.__makeDir( os.path.join( self.paths['render'] ))

        # Nodes dictionary
        self.nodes = dict()

        # Frame attribute dictionary
        self.frames = dict()

        # Set status
        self.status = EngineStatus.none

        # Check Mandatory plugins
        # CHECK OCULA
        # CHECK PIXELACE

        # Check Mandatory nodes
        self.nodes['l_source'] = nuke.toNode( 'LEFT_SOURCE' )
        if self.nodes['l_source'] == None:
            self.__printNullNodeWarning( 'LEFT_SOURCE', self.status )

        self.nodes['r_source'] = nuke.toNode( 'RIGHT_SOURCE' )
        if self.nodes['r_source'] == None:
            self.__printNullNodeWarning( 'RIGHT_SOURCE', self.status )

        self.nodes['source'] = nuke.toNode( 'JOINED_SOURCE' )
        if self.nodes['source'] == None:
            self.__printNullNodeWarning( 'JOINED_SOURCE', self.status )
        else:
            # Load in frame attributes
            self.frames['first'] = int(self.nodes['l_source']['first'].getValue())
            self.frames['last'] = int(self.nodes['l_source']['last'].getValue())


            self.nodes['cameraTracker'] = nuke.toNode( 'CAMERA_TRACKER' )
            if self.nodes['cameraTracker'] == None:
               self.__printNullNodeWarning( 'CAMERA_TRACKER', self.status )
            else:
                # Engine status is confirmed to at tracking stage
                self.status = EngineStatus.track

                self.nodes['camera'] = nuke.toNode( 'TRACKED_CAMERA' )
                if self.nodes['camera'] == None:
                    self.__printNullNodeWarning( 'TRACKED_CAMERA', self.status )
                else:

                    # Engine status is confirmed to at solve stage
                    self.status = EngineStatus.solve

                    self.nodes['solver'] = nuke.toNode( 'SOLVER' )
                    if self.nodes['solver'] == None:
                        __printNullNodeWarning( 'SOLVER', self.status )

                    self.nodes['dispGen'] = nuke.toNode( 'DISPARITY_GENERATOR' )
                    if self.nodes['dispGen'] == None:
                        __printNullNodeWarning( 'DISPARITY_GENERATOR', self.status )





    '''
    Load Methods
    '''
    def load( self ):

        # Setup multi view
        self.__setupMultiView()

        # Delete all nodes except viewer
        allNodes = nuke.allNodes()
        for n in allNodes:
            if n.Class() != 'Viewer':
                nuke.delete(n)

        # Create Left & Right read nodes
        self.nodes['l_source'] = self.__newRead( 'LEFT_SOURCE', self.paths['l_source'] )
        self.nodes['r_source'] = self.__newRead( 'RIGHT_SOURCE', self.paths['r_source'] )


        # Create a joined source node
        self.nodes['source'] = nuke.createNode( 'JoinViews' )
        self.nodes['source'].setName('JOINED_SOURCE')

        self.nodes['source'].setInput( 0, self.nodes['l_source'] )
        self.nodes['source'].setInput( 1, self.nodes['r_source'] )

        # Set their positions so it looks pretty
        self.nodes['l_source'].setXYpos( -50, 0 )
        self.nodes['r_source'].setXYpos( 50, 0 )
        self.nodes['source'].setXYpos( (self.nodes['l_source'].xpos() + self.nodes['r_source'].xpos())/2,
                                  self.nodes['l_source'].ypos() + 100 )

        # Set up root frame ranges
        self.frames['first'] = int( self.nodes['l_source']['first'].getValue() )
        self.frames['last'] = int( self.nodes['l_source']['last'].getValue() )

        nuke.root()['first_frame'].setValue( self.frames['first'] )
        nuke.root()['last_frame'].setValue( self.frames['last'] )



    '''
    Track Methods
    '''

    def track(startFrame, endFrame, increment):

        # Create a CameraTracker and track the source
        t_start_frame = startFrame
        t_end_frame = t_start_frame + self.frames['increment'] - 1
        if (t_end_frame > endFrame):
            print(self.errStr + 'Increment value too high, only tracking to end frame')
            t_end_frame = endFrame

        if self.nodes['l_source'] == None:
            __printNullNodeWarning( 'LEFT_NODE', self.status )

        self.nodes['cameraTracker'] = newCameraTracker( self.nodes['l_source'], t_start_frame, t_end_frame )
        self.nodes['cameraTracker'].setName('CAMERA_TRACKER')

        nuke.show( self.nodes['cameraTracker'] ) # Necessary for execute()

        # Main track loop

        while t_end_frame <= endFrame:

            # Setup project frame ranges
            nuke.root()['first_frame'].setValue( t_start_frame )
            nuke.root()['last_frame'].setValue( t_end_frame )

            # Setup file names
            nuke_script_file_name = self.projectName + '_' + str(t_start_frame) + '_' + str(endFrame) + '.nk'
            script_path = os.path.join(self.paths['track'], nuke_script_file_name)

            # Save new iteration
            nuke.scriptSaveAs(script_path, 1)

            # Prevent camera tracker prompt for overwriting tracks
            self.nodes['cameraTracker']['sequenceIsTracked'].setValue(False)

            # Track 'em!
            self.nodes['cameraTracker']['trackFeatures'].execute()

            # Save
            nuke.scriptSave()

            # Set frame range for next iteration
            t_start_frame = t_end_frame + 1
            t_end_frame = t_start_frame + self.frames['increment'] - 1

            # If t_end_frame is greater than the last frame and t_start_frame is less than last frame
            if t_end_frame > self.frames['last'] and t_start_frame < self.frames['last']:
                t_end_frame = self.frames['last']

            # Set new values for analysis range
            self.nodes['cameraTracker']['analysisStart'].setValue( t_start_frame )
            self.nodes['cameraTracker']['analysisStop'].setValue( t_end_frame )

    '''
    Render Methods
    '''
	def render(self, useCamera, outputInterm, frameRange, dimensions):

        # Check if camera exists

        if useCamera == True:
            if nuke.toNode('CAMERA') == None:
                print(self.errStr + "Camera not found, to render with camera please import a solved camera")
                return

        # Sets up render, callbacks, and begins recurisveRender
        self.__setupRender( outputInterm, 8, dimensions )

        # Set up callbacks
        nuke.addBeforeRender(self.__directoryProtocol)
        nuke.addBeforeFrameRender(self.__pushFirstFrameProtocol)

        # Save it before render
        nuke_script_file_name = self.projectName + '_' + 'render_' + str(self.frames['first']) + '_' + str(self.frames['last']) + '.nk'
        script_path = os.path.join(self.paths['render'], nuke_script_file_name)

        nuke.scriptSaveAs(script_path, 1)

        # Render with Camera
        if useCamera == True:
            nuke.addAfterFrameRender(self.__solverCameraSwitchProtocol)
            self.__recursiveCamRender( frameRange )

        # Render without Camera
        else:
            # Disconnect camera
            try:
                self.nodes['camera']['disable'].setValue(1)
            except KeyError, e:
                #do nothing
                print( self.errStr + 'camera not found. Proceeding...' )

            self.__recursiveNonCamRender( frameRange )

    def __recursiveNonCamRender( self, frameRange ):

        try:
            writeNodes = nuke.allNodes( 'Write' )
            if len( writeNodes ) > 1:
                nuke.executeMultiple( writeNodes, (( frameRange[0], frameRange[1], 1 ),))
            else:
                nuke.execute( nuke.toNode( 'OF_WRITE' ), frameRange[0], frameRange[1], 1)
        except RuntimeError, e:
            # When runtime error occurs, call recursively again with 1 frame increment
            nuke.root()['first_frame'].setValue( int(nuke.root()['first_frame'].getValue()) + 1 )
            self.__recursiveNonCamRender( frameRange )

    def __recursiveCamRender( self, frameRange ):

        # Main render loop
        try:
            writeNodes = nuke.allNodes( 'Write' )
            if len( writeNodes ) > 1:
                nuke.executeMultiple( writeNodes, (( frameRange[0], frameRange[1], 1 ),))
            else:
                nuke.execute( nuke.toNode( 'OF_WRITE' ), frameRange[0], frameRange[1], 1)
        except RuntimeError, e:
            # When runtime error occurs, disable the camera and call recursively again
            self.nodes['camera']['disable'].setValue(1)
            self.__recursiveCamRender()

    def __setupOutput( self, outputID, usePixelace, numberOfViews, dimX, dimY, treeXOffset ):

        newPixelace = 0

        if ( usePixelace ):
            newPixelace = nuke.createNode( 'OFPixelace_1_0' )
            newPixelace.setXYpos( self.nodes['dispGen'].xpos() + treeXOffset, self.nodes['dispGen'].ypos() + 400  )

        # Temporary variables for linking writer nodes
        inputStr = '_VIEW'
        inputNode = 0

        # Check if views need to be scaled to required dimensions
        scaleViews = int(dimX) != self.nodes['dispGen'].width() or int(dimY) != self.nodes['dispGen'].height()

        if scaleViews:
            inputStr = '_OUTPUTCROP'

        self.nodes['outputNodes' + str(outputID)] = list()

        viewCount = int(1)

        while (viewCount <= numberOfViews):

            print('viewCount' + str(viewCount))

            newViewNode = self.nodes['viewList'][viewCount - 1]
            inputNode = newViewNode

            # Make transform if existing format clashes with required format
            if scaleViews:
                viewX = newViewNode.width()
                viewY = newViewNode.height()

                scaleFactorX = float(dimX)/float(viewX)
                scaleFactorY = float(dimY)/float(viewY)

                # Setup transform node
                newTransformNode = nuke.createNode('Transform')
                newTransformNode.setName( 'CAM_' + str( outputID ) + "_" + str( viewCount ) + '_OUTPUTSCALE' )
                newTransformNode.setXYpos( newViewNode.xpos() + treeXOffset, newViewNode.ypos() + 75 )
                newTransformNode.setInput( 0, newViewNode )

                scaleFactor = 1.0

                priorityX = True

                if scaleFactorX > scaleFactorY:
                    scaleFactor = float(scaleFactorX)
                else:
                    scaleFactor = float(scaleFactorY)
                    priorityX = False

                newTransformNode['scale'].setValue(scaleFactor)
                newTransformNode['center'].setValue(0)

                # Setup crop node
                newCropNode = nuke.createNode('Crop')
                newCropNode.setName( 'CAM_' + str( outputID ) + "_" + str( viewCount ) + '_OUTPUTCROP' )
                newCropNode.setXYpos( newTransformNode.xpos(), newTransformNode.ypos() + 75 )
                newCropNode.setInput( 0, newTransformNode )
                newCropNode['reformat'].setValue(1)
                newCropNode['crop'].setValue(1)

                # Algorithm for figuring out crop dimensions
                cropX = 0.0
                cropY = 0.0
                cropR = float( dimX )
                cropT = float( dimY )

                if priorityX:
                    scaledY = int( viewY * scaleFactor + 0.5 )
                    cropEvenOffset = float( scaledY - int( dimY ) ) / 2.0
                    cropY = cropY + cropEvenOffset
                    cropT = float(scaledY) - cropEvenOffset
                else:
                    scaledX = int( viewX * scaleFactor + 0.5 )
                    cropEvenOffset = float( scaledX - int( dimX ) ) / 2.0
                    cropX = cropX + cropEvenOffset
                    cropR = float(scaledX) - cropEvenOffset

                newCropNode['box'].setValue( ( cropX, cropY, cropR, cropT ) )

                inputNode = newCropNode


            self.nodes['outputNodes' + str(outputID)].append(inputNode)

            if (not usePixelace):
                # Make write nodes for multi view rendering
                newWriteNode = nuke.createNode( 'Write' )
                newWriteNode.setName( 'CAM_' + str(viewCount) + '_WRITE' )
                newWriteNode.setInput( 0, inputNode )
                newWriteNode['views'].setValue('left')
                newWriteNode.setXYpos( inputNode.xpos(), inputNode.ypos() + 35 )
                newWriteNode['file'].fromUserText(os.path.join(self.paths['output'], str(dimX) + '_' + str(dimY) + '_interm', 'cam' + str(viewCount),
                                      self.projectName + '_cam' + str(viewCount) + '_' + str(dimX) + '_' + str(dimY) + '_%07d.tga' ))

            # Increment viewcount
            viewCount = viewCount + 1

        # Write node for Pixelace
        if ( usePixelace ):
            newWriteNode = nuke.createNode( 'Write' )
            newWriteNode.setName( 'OF_WRITE' )
            newWriteNode.setInput( 0, newPixelace )
            newWriteNode['views'].setValue( 'left' )
            newWriteNode.setXYpos(newPixelace.xpos(), newPixelace.ypos() + 100 )

            # Build proper directory path
            newWriteNode['file'].fromUserText(os.path.join(self.paths['output'], str(dimX) + '_' + str(dimY),
                                          self.projectName + '_' + str(dimX) + '_' + str(dimY) + '_%07d.tga' ))

            # Connect pixelace
            inputCount = 0
            while inputCount < numberOfViews:
                newPixelace.setInput( inputCount, self.nodes['outputNodes' + str(outputID)][inputCount])
                inputCount = inputCount + 1

    def __setupRender( self, outputInterm, numberOfViews, dimensions ):

        # Setup render nodes

        oculaVerStr = '2_1'

        if ( not nuke.oculaPresent() ):
            print ( self.errStr + 'Ocula ' + oculaVerStr + ' Plugin not present' )
            return

        # Delete any existing solver, dgen, new view, pixelace, or write nodes

        self.__deleteNodesWithType( 'O_Solver' + oculaVerStr )
        self.__deleteNodesWithType( 'O_DisparityGenerator' + oculaVerStr )
        self.__deleteNodesWithType( 'O_NewView' + oculaVerStr )
        self.__deleteNodesWithType( 'OFPixelace_1_0' )
        self.__deleteNodesWithType( 'Write' )

        # Try to find camera tracker
        try:
            print( self.nodes['cameraTracker'] )
        except KeyError, e:
            #if tracker node not found return with error
            print ( self.warnStr + 'CameraTracker node not found, unable to gurrantee root range authenticy' )

        # Begin by creating solver node, setting it's attributes and positioning it
        self.nodes['solver'] = nuke.createNode( 'O_Solver' + oculaVerStr )
        self.nodes['solver'].setName( 'SOLVER' )
        self.nodes['solver'].setInput( 0, self.nodes['source'] ) #source
        try:
            self.nodes['solver'].setInput( 2, self.nodes['camera'] ) #camera
        except KeyError, e:
            print( self.errStr + 'TRACKED_CAMERA not found, continuing set up' )
        self.nodes['solver'].setXYpos( self.nodes['source'].xpos(), self.nodes['source'].ypos() + 50 )

        # Setup Disparity Generator
        self.nodes['dispGen'] = nuke.createNode( 'O_DisparityGenerator' + oculaVerStr )
        self.nodes['dispGen'].setName( 'DISPARITY_GENERATOR' )
        self.nodes['dispGen'].setInput( 0, self.nodes['solver'] ) #source
        self.nodes['dispGen'].setInput( 1, self.nodes['solver'] ) #solver

        # Position disparity generator below solver
        self.nodes['dispGen'].setXYpos( self.nodes['solver'].xpos(), self.nodes['solver'].ypos() + 100 )

        # Setup views

        self.nodes['viewList'] = list()

        viewCount = 1

        while (viewCount <= numberOfViews):

            # Create new view node then set it up
            newViewNode = nuke.createNode('O_NewView' + oculaVerStr)

            newViewNode.setInput( 0, self.nodes['dispGen'] )
            newViewNode.setName('CAM_' + str( viewCount ) + '_VIEW')
            newViewNode['interpolatePosition'].setValue( 1.0 - ( float(viewCount - 1.0) / float( numberOfViews - 1.0 )))

            newViewNode.setXYpos( self.nodes['dispGen'].xpos() + ( float( viewCount ) - float( numberOfViews/2 ) - 0.5 ) * 100, self.nodes['dispGen'].ypos() + 100)

            self.nodes['viewList'].append(newViewNode)

            viewCount = viewCount + 1

        # Setup Output nodes

        outputNum = len( dimensions ) - 1
        if outputInterm:
            outputNum = outputNum + 1

        outputTreeSingleLen = 1000
        outputTreeTotalLen = outputNum * outputTreeSingleLen
        outputTreeStartX = 0.0 - float(outputTreeTotalLen)/2

        outputCount = 0

        while outputCount < len( dimensions ):
            print( 'Output Dimensions: ' + str(dimensions[outputCount]))
            self.__setupOutput( outputCount, True, numberOfViews, dimensions[outputCount][0], dimensions[outputCount][1], outputTreeStartX + ( outputCount * outputTreeSingleLen ))

            outputCount = outputCount + 1

        if outputInterm:
            self.__setupOutput( outputCount, False, numberOfViews, 640, 400, outputTreeStartX + ( outputCount * outputTreeSingleLen ))


	'''
	Nuke Callback Methods
	'''

	def __directoryProtocol( self ):

		thisNode = nuke.thisNode()

		if ( thisNode.Class() == 'Write' ):

			# Get directory path
			filePath = thisNode['file'].getValue()
			dirPath = os.path.dirname( filePath )

			self.__makeDir(dirPath)

    # Before & After frame render calls is for the rendering stage, it disconnects the camera node from the
    # solver when insufficient feature pts error occurs and connects otherwise

    def __pushFirstFrameProtocol( self ):
        #Keep setting the first frame to the current frame to push render, shot ranges are stored in cameraTracker node
        nuke.root()['first_frame'].setValue( nuke.root()['frame'].getValue() )

    def __solverCameraSwitchProtocol( self ):
        thisNode = nuke.allNodes( 'O_Solver2_1' )[0]

        # Check if node is a solver node
        if thisNode.Class() == 'O_Solver2_1':
            thisNode.redraw()

            if thisNode.input(0) != None:
                nodes['camera'] = nuke.allNodes( 'Camera' )[0]

                if not thisNode.hasError():
                    # if solver has no error and camera is disabled, enable it
                    if nodes['camera']['disable'].getValue() == 1:
                        nodes['camera']['disable'].setValue( 0 )


    '''
    Error Methods
    '''

    def __printNullNodeWarning( self, nodeName, stage ):

        # Error printer
        errorString = ''

        if stage == EngineStatus.none:
            errorString = ' not found, commence load(), then track() if camera is required'
        elif stage == EngineStatus.track:
            errorString = ' not found, commence solve()'
        elif stage == EngineStatus.solve:
            errorString = ' not found, commence render()'

        print (self.warnStr + nodeName + errorString )

# main protocol

_ttmGlobal = ttmEngineController()
