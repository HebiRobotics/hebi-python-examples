import hebi
import numpy
from time import sleep
import tkinter
from pyquaternion import Quaternion


class Mesh:
    '''
    This class describes a 3D mesh that is used to represent the robot components
    '''

    def __init__(self, vertices, edges, faces, face_vectors):
        # vertices is a list of points in a mesh
        self.vertices = vertices
        # edges is a list of edges that connect adjacent vertices
        self.edges = edges
        # faces are a list of faces in the mesh. faces can contain 3 or more points
        self.faces = faces
        # face_vectors is a list of unit vectors that are parellel to the corresponding faces. These are used for shading
        self.face_vectors = face_vectors


'''
these "base" meshes are parameterizable meshes that represent the base shapes before any transformations have been 
applied. the "@1,@2..." are the parametric variables. These are replaced by the values of your choice by parse_mesh (see
below). The first is a cube and the second is a rectangle of variable length
'''
base_meshes = {'actuator': Mesh(vertices=[['-1*@1', '-1*@1', '@1'], ['-1*@1', '-1*@1', '-1*@1'],
                                          ['-1*@1', '@1', '@1'], ['-1*@1', '@1', '-1*@1'],
                                          ['@1', '@1', '-1*@1'], ['@1', '@1', '@1'],
                                          ['@1', '-1*@1', '@1'], ['@1', '-1*@1', '-1*@1']],
                                edges=[(0, 1), (0, 6), (0, 2), (1, 7), (1, 3), (2, 3), (2, 5), (3, 4), (4, 5), (4, 7),
                                       (5, 6), (6, 7)],
                                faces=[(0, 1, 3, 2), (0, 1, 7, 6), (0, 2, 5, 6), (2, 3, 4, 5), (1, 3, 4, 7),
                                       (4, 5, 6, 7)],
                                face_vectors=[(-1, 0, 0), (0, -1, 0), (0, 0, 1), (0, 1, 0), (0, 0, -1), (1, 0, 0)]),

               'link': Mesh(vertices=[[0, '-1*@1', '@1'], [0, '-1*@1', '-1*@1'],
                                      [0, '@1', '@1'], [0, '@1', '-1*@1'],
                                      ['@2', '@1', '-1*@1'], ['@2', '@1', '@1'],
                                      ['@2', '-1*@1', '@1'], ['@2', '-1*@1', '-1*@1']],
                            edges=[(0, 1), (0, 6), (0, 2), (1, 7), (1, 3), (2, 3), (2, 5), (3, 4), (4, 5), (4, 7),
                                   (5, 6), (6, 7)],
                            faces=[(0, 1, 3, 2), (0, 1, 7, 6), (0, 2, 5, 6), (2, 3, 4, 5), (1, 3, 4, 7), (4, 5, 6, 7)],
                            face_vectors=[(-1, 0, 0), (0, -1, 0), (0, 0, 1), (0, 1, 0), (0, 0, -1), (1, 0, 0)])}


# this is used to draw models of robots with a virtual position
class VirtualFeedback:
    def __init__(self, position,orientation):
        self.position = position  # the virtual positions of the robot
        self.orientation = orientation  # this is the identity quaternion


'''
the Robot class represents your robot. specific values for the robot model are defined in __init__ and can be changed
according to your specific configuration
'''


class Robot:
    # constructs a hebi robot model from a list of components
    def make_model(self, components):
        model = hebi.robot_model.RobotModel()
        for c in components:
            if c[0] == 'actuator':
                model.add_actuator((c[1]))
            elif c[0] == 'bracket':
                model.add_bracket(c[1], c[2])
            elif c[0] == 'link':
                model.add_link(c[1], c[2], c[3])
        return model

    def __init__(self):
        # finds available actuators on network
        lookup = hebi.Lookup()
        # Give the Lookup process 2 seconds to discover modules
        sleep(2)
        self.group = lookup.get_group_from_names(['HEBI'], ['X5-9', 'X-00147']) #change depending on
        self.group.feedback_frequency = 24.0
        # this is a list of components that can be iterated through that represent the robot
        # i.e. components[0] is linked to components[1] etc.
        self.components = [['actuator', 'X5-1'], ['bracket', 'X5-LightBracket', 'right'], ['link', 'X5', 5.0, 0],
                           ['actuator', 'X5-1'], ['link', 'X5', 5.0, 0]]
        self.model = self.make_model(self.components)
        self.mesh = []
        self.base_rotation=[1, 0, 0, 0]

    # parse mesh substitutes parametric variables into the base mesh so vals[0] replaces @1, vals[1] replaces @2 etc.
    # note that the tags for parametric variables are 1 indexed and vals are 0 indexed
    def parse_mesh(self, m, vals):
        v = m.vertices
        for i in range(len(vals)):
            v = [[str(x).replace('@' + str(i + 1), str(vals[i])) for x in v[k]] for k in range(len(m.vertices))]
        v = [[eval(x) for x in v[k]] for k in range(len(v))]
        return Mesh(v, m.edges, m.faces, m.face_vectors)

    #used as a callback for the virtual robot to keep bases alligned
    def set_base_rotation(self, feedback):
        self.base_rotation = feedback.orientation

    # this functions uses forward kinematics to transform base meshes into the position of the robot component
    # both vertices and face_vectors are transformed so that the shading is consistent with the model position
    # this function is called by the feedback callback (this is assigned in the app class)
    def generate_3d_mesh(self, feedback):
        self.mesh = []  # creates empty list of meshes
        angles = feedback.position  # get the angles of the actuators
        base_rotation = Quaternion(feedback.orientation[0]).transformation_matrix  # determines rotation of the base
        transforms = self.model.get_forward_kinematics('output', angles)
        # adds an identity matrix to the list of transforms for consistent indexing (otherwise the first transform would
        # refer to the position of the second component
        transforms.insert(0, numpy.matrix([[1, 0, 0, 0], [0, 1, 0, 0], [0, 0, 1, 0], [0, 0, 0, 1]]))
        # multiplies base rotation and the kinematic transforms (unless there are nan values in the quaternion)
        if not numpy.isnan(numpy.array(feedback.orientation[0],dtype=numpy.float64)).any():
            transforms = [numpy.dot(base_rotation, transforms[i]) for i in range(len(self.components))]
        for i in range(len(self.components)):
            if self.components[i][0] == 'bracket':  # no model for the bracket
                continue
            m = Mesh([], [], [], [])
            bm = Mesh(base_meshes[self.components[i][0]].vertices, base_meshes[self.components[i][0]].edges,
                      base_meshes[self.components[i][0]].faces, base_meshes[self.components[i][0]].face_vectors)
            if self.components[i][0] == 'link':
                bm = self.parse_mesh(bm, [.5, self.components[i][2]])
            if self.components[i][0] == 'actuator':
                bm = self.parse_mesh(bm, [.75])
            m.edges = bm.edges  # edges and faces are the same for base and transformed meshes
            m.faces = bm.faces
            for j in range(len(bm.vertices)):
                # append one to end of vertex to make matrix multiplication possible (transformation matrices are 4x4
                v = [bm.vertices[j][0], bm.vertices[j][1], bm.vertices[j][2], 1]
                vT = numpy.dot(transforms[i], v)  # apply transform to vertex
                m.vertices.append(vT.A1[0:3])  # add transformed vertex to mesh
            for j in range(len(bm.face_vectors)):  # similar but for face vectors
                normal = bm.face_vectors[j]
                TRot = transforms[i][numpy.ix_([0, 1, 2], [0, 1, 2])]
                nT = numpy.dot(TRot, normal)
                m.face_vectors.append(nT.A1)
            self.mesh.append(m)

'''
the App class is what actually creates the window, runs the program and takes in user inputs
uses tkinter library
'''
class App:
    def __init__(self, title):
        self.window = tkinter.Tk()  # creates a tkinter window
        self.window.title(title)
        self.robot = Robot()  # makes initializes a robot
        self.robot.group.add_feedback_handler(self.robot.generate_3d_mesh)  # adds generate mesh to feedback handler
        self.virtual_robot = Robot()
        self.virtual_robot.group.add_feedback_handler(self.virtual_robot.set_base_rotation)
        self.camera = [0, 25, 3]  # default camera position
        self.camera_angle = -1 * numpy.pi / 2  # default camera angle

    #this iterates through the robot components and makes a slider (Scale) for each actuator and makes a list for future
    #reference
    def add_sliders(self):
        feedback = self.robot.group.get_next_feedback()
        self.sliders = []
        j = 0
        for i in range(len(self.robot.components)):
            if self.robot.components[i][0] == 'actuator':
                w = (tkinter.Scale(self.window, from_=-2 * numpy.pi, to=2 * numpy.pi, orient='horizontal', length=400,
                                   resolution=.01, ), i)
                w[0].set(feedback.position[j])
                w[0].pack(side="top", )
                self.sliders.append(w)
                j += 1
    #creates canvas for drawing mesh onto and places it at the bottom of the window
    def begin_canvas(self, h, w):
        self.canvas = tkinter.Canvas(self.window, width=w, height=h)
        self.canvas.pack(side="bottom")

    #function is used to move the camera (namely by user input, see bind_keys below
    def move_camera(self, move, angle):
        TRot = [[1, 0, 0], [0, numpy.cos(self.camera_angle), -1 * numpy.sin(self.camera_angle)],
                [0, numpy.sin(self.camera_angle), numpy.cos(self.camera_angle)]]
        move = numpy.dot(TRot, move)
        self.camera = (numpy.array(self.camera) - numpy.array(move)).tolist()
        self.camera_angle += angle

    #places camera back at default position
    def center_camera(self, event):
        self.camera_angle = -1 * numpy.pi / 2
        self.camera = [0, 25, 3]

    #determines the 3D center of a face (used for determining draw order)
    def calculate_face_center(self, mesh, i):
        totals = [0, 0, 0]
        num = 0
        for p in mesh.faces[i]:
            totals[0] += mesh.vertices[p][0]
            totals[1] += mesh.vertices[p][1]
            totals[2] += mesh.vertices[p][2]
            num += 1
        return [t / num for t in totals]

    #Determined the color of the face by calculating the projection of the face vector and the incident vector (vector
    #from light source to face center
    def generate_shading(self, face_vector, face_pos, light_src, base_color):
        face_vector = numpy.array(face_vector)
        face_pos = numpy.array(face_pos)
        light_src = numpy.array(light_src)
        incident_vector = light_src - face_pos
        proj_inc_face = (numpy.dot(face_vector, incident_vector)) * incident_vector / (
                numpy.linalg.norm(incident_vector) ** 2)
        color = [str(hex(int(base_color[i] * numpy.linalg.norm(proj_inc_face)))).replace('0x', '') for i in range(3)]
        for i in range(3):
            if len(color[i]) < 2:
                color[i] = "0" + color[i]
        return "#" + "".join(color)

    #draws mesh as a wireframe by applting a perspective transform on the 3d point to translate it to 2d
    #also applies transformation for the camera rotation and position
    #offset is the offset of the image, not the object (needed so it is not drawn at (0,0))
    def draw_mesh_wireframe(self, mesh, colors, horizon, offsets, camera, camera_angle):
        i = 0
        for m in mesh:
            for e in m.edges:
                v1 = [x - y for x, y in zip(m.vertices[e[0]], camera)]
                TRot = [[1, 0, 0], [0, numpy.cos(camera_angle), -1 * numpy.sin(camera_angle)],
                        [0, numpy.sin(camera_angle), numpy.cos(camera_angle)]]
                v1 = numpy.dot(TRot, v1)
                transform1 = [[-1 * horizon / v1[2], 0, 0], [0, -1 * horizon / v1[2], 0]]
                p1 = numpy.dot(transform1, v1)
                p1[0] += offsets[0]
                p1[1] += offsets[1]
                v2 = [x - y for x, y in zip(m.vertices[e[1]], camera)]
                v2 = numpy.dot(TRot, v2)
                transform2 = [[-1 * horizon / v2[2], 0, 0], [0, -1 * horizon / v2[2], 0]]
                p2 = numpy.dot(transform2, v2)
                p2[0] += offsets[0]
                p2[1] += offsets[1]
                self.canvas.create_line(p1[0], p1[1], p2[0], p2[1], fill=colors[i % len(colors)])
            i += 1

    #draws mesh as a solid using a similar technique, however it differs in that it calculates the position of each face
    #first, sorts them by distance from camera and draws in that order (this is so objects appear at the correct depth
    def draw_mesh_solid(self, mesh, mesh_colors, horizon, offsets, camera, camera_angle, light):
        color_iterator = 0
        faces = []
        colors = []
        depths = []
        for m in mesh:
            for i in range(len(m.faces)):
                points = []
                pos = self.calculate_face_center(m, i)
                depths.append(numpy.linalg.norm(numpy.array(pos) - numpy.array(camera)))
                colors.append(self.generate_shading(m.face_vectors[i], pos, light,
                                                    mesh_colors[color_iterator % len(mesh_colors)]))
                for p in m.faces[i]:
                    v = [x - x0 for x, x0 in zip(m.vertices[p], camera)]
                    TRot = [[1, 0, 0], [0, numpy.cos(camera_angle), -1 * numpy.sin(camera_angle)],
                            [0, numpy.sin(camera_angle), numpy.cos(camera_angle)]]
                    v = numpy.dot(TRot, v)
                    transform = [[-1 * horizon / v[2], 0, 0], [0, -1 * horizon / v[2], 0]]
                    point = numpy.dot(transform, v)
                    point[0] += offsets[0]
                    point[1] += offsets[1]
                    points.append(point)
                faces.append(points)
            color_iterator += 1
        faces = [x for _, x in sorted(zip(depths, faces), key=lambda pair: pair[0])]
        faces.reverse()
        colors = [x for _, x in sorted(zip(depths, colors), key=lambda pair: pair[0])]
        colors.reverse()
        for i in range(len(faces)):
            vects = []
            for j in range(len(faces[i])):
                vects.extend(faces[i][j].tolist())
            self.canvas.create_polygon(vects, fill=colors[i])

    def clear_canvas(self):
        self.canvas.delete(tkinter.ALL)

    #this is functions updates all the values in the app and robot and sends slider commands to robot
    def update(self):
        self.clear_canvas()
        group_command = hebi.GroupCommand(self.robot.group.size)
        p = []
        for w in self.sliders:
            p.append(w[0].get())
        group_command.position = p
        self.robot.group.send_command(group_command)
        self.virtual_robot.generate_3d_mesh(VirtualFeedback(p,self.virtual_robot.base_rotation))
        self.draw_mesh_wireframe(self.virtual_robot.mesh, colors=['yellow', 'yellow', 'yellow', 'yellow']
                                 , horizon=400, offsets=[250, 250, 0], camera=self.camera,
                                 camera_angle=self.camera_angle)
        self.draw_mesh_solid(self.robot.mesh, mesh_colors=[[0, 255, 0], [0, 0, 255]]
                             , horizon=400, offsets=[250, 250, 0], camera=self.camera, camera_angle=self.camera_angle,
                             light=[0, 10, 10])
    #binds keys to move camera
    def bind_keys(self):
        self.window.bind("<Left>", lambda event, move=(-1, 0, 0), angle=0: self.move_camera(move, angle))
        self.window.bind("<Right>", lambda event, move=(1, 0, 0), angle=0: self.move_camera(move, angle))
        self.window.bind("<Up>", lambda event, move=(0, 1, 0), angle=0: self.move_camera(move, angle))
        self.window.bind("<Down>", lambda event, move=(0, -1, 0), angle=0: self.move_camera(move, angle))
        self.window.bind("<w>", lambda event, move=(0, 0, 1), angle=0: self.move_camera(move, angle))
        self.window.bind("<s>", lambda event, move=(0, 0, -1), angle=0: self.move_camera(move, angle))
        self.window.bind("<e>", lambda event, move=(0, 0, 0), angle=-0.314: self.move_camera(move, angle))
        self.window.bind("<d>", lambda event, move=(0, 0, 0), angle=0.314: self.move_camera(move, angle))
        self.window.bind("<c>", self.center_camera)

    #this function repeatedly calls the update function to keep the program up to data ~60Hz (1/60 ~ .016s
    def clock(self):
        # lab['text'] = time
        self.update()
        self.window.after(16, self.clock)

#creates the app, sliders, canvas and clock, then runs the app
if __name__ == "__main__":
    gui = App("test")
    gui.begin_canvas(500, 500)
    gui.add_sliders()
    gui.bind_keys()
    gui.clock()
    gui.window.mainloop()
