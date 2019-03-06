from dolfin import *
from fem import mesh_reader, domain_dict, vector_converter
import pyqtgraph.opengl as gl
import matplotlib.cm as cmx
import numpy as np
import os
import time



class FEM_Solver:
    root_dir = './fem_data/'
    mesh_dir = root_dir + 'mesh/'
    refcap_dir = root_dir + 'refcap/'
    measured_dir = root_dir  + 'measured/'

    def __init__(self, mesh_filename, subdomain=False, boundary=False):
        print("[FEM Solver initialization start]")
        init_time_start = time.time()
        #
        self.convert_xml_to_hdf5_flag = False
        #
        ############### Check fem_data system directory ###############
        if not os.path.isdir(FEM_Solver.root_dir):
            print("Make default fem_data root directory: ./fem_data")
            os.mkdir(FEM_Solver.root_dir)
        #
        ############### Listing mesh directory ###############
        mesh_files = []
        for file in os.listdir(FEM_Solver.mesh_dir):
            mesh_files.append(file)
        #
        ############### Search mesh file in fem_data directory ###############
        # Detect XML mesh file first
        if (mesh_filename + '.xml') in mesh_files:
            print("XML mesh file detected")
            mesh_file_format = 'xml'
            if self.convert_xml_to_hdf5_flag:
                print("Converting XML to HDF5")
                hdf5_filename = mesh_reader.xml_to_hdf5(FEM_Solver.mesh_dir, mesh_filename, subdomain, boundary)
        # If there is no XML mesh file, then detect hdf5 mesh file
        elif (mesh_filename + '.h5') in mesh_files:
            print("HDF5 mesh file detected")
            mesh_file_format = 'hdf5'
        else:
            raise Exception('No mesh file (XML or HDF5 format) has been was detected')
        #
        #
        ############### Load mesh & physical marker files ###############
        print("Loading mesh file...")
        time_start = time.time()
        #
        if mesh_file_format == 'xml':
            (self.mesh, self.subdomains, self.boundaries) = mesh_reader.read_xml_mesh(FEM_Solver.mesh_dir, mesh_filename, subdomain, boundary)
        elif mesh_file_format == 'hdf5':
            (self.mesh, self.subdomains, self.boundaries) = mesh_reader.read_hdf5_mesh(FEM_Solver.mesh_dir, mesh_filename, subdomain, boundary)
        #
        print("elapsed time = {} sec".format(time.time() - time_start))
        #
        #
        ############### Set physical marker & material dictionaries ###############
        (self.subdomain_dict, self.boundary_dict) = domain_dict.markers()
        (k_dict,) = domain_dict.materials(self.subdomain_dict)
        #
        #
        ############### Define material characteristics - dielectric constant (k) ###############
        print("Loading permittivity map file...")
        time_start = time.time()
        #
        self.k = self.set_permittivity(self.mesh, self.subdomains, k_dict)
        #
        print("elapsed time = {} sec".format(time.time() - time_start))
        #
        #
        ############### Define dx & ds 'Measure instance' for domains & boundaries ###############
        dx = Measure('dx', subdomain_data=self.subdomains)
        ds = Measure('ds', subdomain_data=self.boundaries)
        #
        #
        ############### Define PDE Problem for electrostatic potential distribution (phi) ###############
        print("Define PDE problems - Phi")
        time_start = time.time()
        #
        (self.V, self.phi, self.a, self.L) = self.set_problem_phi(self.k, dx, ds, self.mesh)
        #
        print("elapsed time = {} sec".format(time.time() - time_start))
        #
        #
        ############### Define Post-processing PDE problem ###############
        print("Define PDE problems - grad_Phi")
        time_start = time.time()
        #
        (self.V_grad, self.grad_phi, self.a_grad, self.L_grad) = self.set_problem_phi_grad(dx, self.phi, self.mesh)
        #
        print("elapsed time = {} sec".format(time.time() - time_start))
        #
        #
        ############### Set solver ###############
        self.solver_params = self.set_solver()
        #
        #
        ############### Extract cell_volumes, cell_coordinates, dof_index_to_cell_tables ###############
        (self.cell_volumes, self.cell_coordinates, self.dof_index_to_cell_table) = vector_converter.convert_vector_nodes_to_cells(FEM_Solver.root_dir, self.mesh, self.V_grad)
        #
        #
        ############### Make dof_index_to_cell_table to 1D array ###############
        self.dof_index_to_cell_1D = self.dof_index_to_cell_table.flatten()
        #
        #
        ############### Pre-define some constants ###############
        self.sensitivity_map = None
        self.total_cell_num = self.mesh.num_cells()
        self.dofs_num_per_cell = len(self.V_grad.dofmap().cell_dofs(0))
        self.V_exc = None
        self.total_electrode_num = None
        self.unit_scale = 1         # Mesh base scale => 1 : m, 1000: mm
        #
        ############### Pre-define C_min & C_max array ###############
        self.Cmin = None
        self.Cmax = None
        #
        #
        ############### Get air cells info ###############
        self.air_cell_indices = vector_converter.get_air_cell_indices(FEM_Solver.root_dir, self.subdomains, self.subdomain_dict)
        self.sensing_domain_coordinates = self.cell_coordinates[self.air_cell_indices]
        self.total_air_cell_num = self.air_cell_indices.size
        #
        #
        # Get face coordinates
        # self.face_array = vector_converter.calculate_face_array(self.mesh)
        #
        #
        init_time_stop = time.time()
        print("FEM Solver initialization complete (total {} sec)".format(init_time_stop - init_time_start))

    def set_number_of_electrodes(self, electrode_number):
        self.total_electrode_num = electrode_number

    def set_exc_voltage(self, voltage):
        self.V_exc = voltage

    def set_exc_electrode(self, elec_num):
        exc_electrode = 'elec' + str(elec_num+1)
        # Define Dirichlet boundary condition for the excitation electrode surface: V = V_exc
        bcs = [DirichletBC(self.V, Constant(self.V_exc), self.boundaries, self.boundary_dict.get(exc_electrode))]
        # Define Dirichlet boundary condition for ground Cu surfaces
        gnd_surface_dict = self.boundary_dict
        del gnd_surface_dict[exc_electrode]     # Exclude excitation electrode surfaces from boundary conditions
        if 'air' in gnd_surface_dict:
            del gnd_surface_dict['air']             # Exclude air surfaces from boundary conditions
            print("'air' boundaries has been excluded from FEM boundary conditions")
        for val in gnd_surface_dict.values():
            bcs.append(DirichletBC(self.V, 0.0, self.boundaries, val))
        return bcs

    def get_Cmin(self, Cmin_filename):
        Cmin = np.loadtxt(FEM_Solver.refcap_dir + Cmin_filename)
        Cmin = Cmin.reshape((Cmin.size, 1))
        self.Cmin = Cmin

    def get_Cmax(self, Cmax_filename):
        Cmax = np.loadtxt(FEM_Solver.refcap_dir + Cmax_filename)
        Cmax = Cmax.reshape((Cmax.size, 1))
        self.Cmax = Cmax

    def set_permittivity(self, mesh, subdomains, k_dict, family='DG', degree=0):
        # Define function space for material parameter
        V0 = FunctionSpace(mesh, family, degree)
        k = Function(V0)
        file_path = FEM_Solver.root_dir + 'k_vector.npy'
        if os.path.isfile(file_path):
            k.vector()[:] = np.load(file_path)
        else:
            print("No permittivity map file detected")
            print(" ==> Creating new permittivity map")
            k.vector()[:] = np.array([k_dict.get(key) for key in subdomains.array()[:]])
            np.save(file_path, k.vector()[:])
        return k

    def set_problem_phi(self, k, dx, ds, mesh, family='CG', degree=1):
        # Define the scalar nodal function space (V) as a Lagrange function space
        # With degree 1 (scalar_order), we get the linear Lagrange element
        V = FunctionSpace(mesh, family, degree)
        # Create function instance for solution - phi (electrostatic potential)
        phi = Function(V)
        # Define the test and trial functions from the V, with u and v the Lagrange basis functions
        (u, ) = TrialFunctions(V)
        (v, ) = TestFunctions(V)
        # rho=0 (constant) for the electrostatic Laplace form of Poisson equation
        rho = Constant(0.0)
        # For Robin boundary condition
        # p is calculated as approximate boundary condition d(phi)/dr = 1/(r*ln(r))*phi
        # ln(r) has been implemented as log10(r)/log10(e), because of the log(x) syntax bug
        p = Expression('1/(sqrt(x[0]*x[0] + x[1]*x[1] + x[2]*x[2])*log10(sqrt(x[0]*x[0] + x[1]*x[1] + x[2]*x[2]))/log10(exp(1)))', degree=3)
        # q is as a source
        q = Constant(0.0)
        # Specify PDE problem
        a = k*inner(nabla_grad(u), nabla_grad(v))*dx + k*p*u*v*ds(self.boundary_dict.get('air'))
        L = rho*v*dx + p*q*v*ds(self.boundary_dict.get('air'))
        # Return FunctionSpace (V) & Function (phi) instances,         self.define_measure()a & L
        return (V, phi, a, L)

    def set_problem_phi_grad(self, dx, phi, mesh, family='P', degree=1):
        V_grad = VectorFunctionSpace(mesh, family, degree)
        grad_phi = Function(V_grad)
        w_grad = TrialFunction(V_grad)
        v_grad = TestFunction(V_grad)
        a_grad = inner(w_grad, v_grad) * dx
        L_grad = inner(grad(phi), v_grad) * dx
        return (V_grad, grad_phi, a_grad, L_grad)

    def set_solver(self, solver='krylov_solver', abs_tol=1E-11, rel_tol=1E-7, max_iter=100000):
        if solver == 'krylov_solver':
            params = parameters[solver]
            params['absolute_tolerance'] = abs_tol
            params['relative_tolerance'] = rel_tol
            params['maximum_iterations'] = max_iter
        else:
            raise Exception('Other solvers have not been implemented yet')
        return {
        'linear_solver': 'gmres',
        'preconditioner': 'ilu',
        'krylov_solver': params}

    def solve_phi(self, elec_num):
        bcs = self.set_exc_electrode(elec_num)
        solve(self.a == self.L, self.phi, bcs, solver_parameters=self.solver_params)

    def solve_grad_phi(self, elec_num):
        # Solve the PDE problem for phi first
        self.solve_phi(elec_num)
        # Solve the PDE problem for grad(phi)
        solve(self.a_grad == self.L_grad, self.grad_phi, solver_parameters=self.solver_params)
        # Get grad_phi values for each cell
        grad_phi_per_cell_table = self.grad_phi.vector()[self.dof_index_to_cell_1D].reshape((self.total_cell_num, self.dofs_num_per_cell))
        # Split grad_phi array into grad_phi_Xs, grad_phi_Ys, grad_phi_Zs
        # Calculate average of each grad_phi_Axis_values (Shape of each axis is transposed)
        # self.grad_phi_cells[0] = d(phi)/d(x)
        # self.grad_phi_cells[1] = d(phi)/d(y)
        # self.grad_phi_cells[2] = d(phi)/d(z)
        grad_phi_cells = np.mean(np.split(grad_phi_per_cell_table, 3, axis=1), axis=2, dtype=np.float64)
        return grad_phi_cells

    def get_grad_phi_dict(self, progress_bar=None):
        # Pre-define grad(phi) array for all electrodes
        # self.grad_phi_cells = np.empty((self.total_electrode_num, self.total_cell_num))
        self.grad_phi_vector_dict = {}
        for elec_num in range(self.total_electrode_num):
            print("***** Get grad_phi from electrode {}... *****".format(elec_num+1))
            grad_phi_vector = self.solve_grad_phi(elec_num)
            self.grad_phi_vector_dict.update({elec_num:grad_phi_vector})
            if not progress_bar == None:
                progress_bar.setValue(int((elec_num+1) / self.total_electrode_num * 100))

    def calculate_sensitivity_map(self, progress_bar=None):
        print("Loading sensitivity map file...")
        smap_timer_start = time.time()
        file_path = FEM_Solver.root_dir + 'sensitivity_map.npy'
        if os.path.isfile(file_path):
            self.sensitivity_map = np.load(file_path)
            if not progress_bar == None:
                progress_bar.setValue(100)
        else:
            print("No sensitivity map file detected")
            print(" ==> Creating new sensitivity map")
            self.get_grad_phi_dict(progress_bar)
            combination_num = int(self.total_electrode_num * (self.total_electrode_num - 1) / 2)
            self.sensitivity_map = np.empty((combination_num, self.total_cell_num))
            cnt = 0
            for i in range(self.total_electrode_num):
                for j in range(i + 1, self.total_electrode_num):
                    print("Calculate S_{i}{j}".format(i=i,j=j))
                    inner = self.grad_phi_vector_dict[i] * self.grad_phi_vector_dict[j]
                    self.sensitivity_map[cnt] = (-1.0/(self.V_exc*self.V_exc)) * self.cell_volumes * np.sum(inner, axis=0)
                    cnt += 1
            np.save(file_path, self.sensitivity_map)
        smap_timer_stop = time.time()
        print("Completed - elapsed time = {:f} sec".format(smap_timer_stop - smap_timer_start))

    def calculate_forward_cap_array(self):
        e0 = 8.85418781762039 * 0.000000000001  # Unit: F/m
        self.calculate_sensitivity_map()
        C_forward = e0 * np.matmul(self.sensitivity_map, self.k.vector()[:])
        return C_forward

    ####################################################################################################################
    def debug_plot_image(self, epsilon, cmap, base_opacity, norm_method, opacity_filter, opacity_param=0.1):
        self.epsilon_sensing_domain = epsilon[self.air_cell_indices]
        self.debug_solver_viewer(self.sensing_domain_coordinates,
                                 self.epsilon_sensing_domain,
                                 cmap=cmap,
                                 norm_method=norm_method,
                                 gamma=0.5,
                                 base_opacity=base_opacity,
                                 opacity_filter=opacity_filter,
                                 opacity_param=opacity_param)
        self.debug_tracker()

    def debug_solver_viewer(self, coordinates, value, cmap, norm_method, gamma, base_opacity, opacity_filter=None, opacity_param=0.1):
        #
        import image_filter
        self.image_filter = image_filter.ImageFilter()
        #
        self.gl_widget = gl.GLViewWidget()
        self.gl_widget.show()
        self.gl_widget.setWindowTitle('FEM Solver Viewer')
        self.gl_widget.setCameraPosition(distance=0.05*self.unit_scale)
        # Add Grid Item
        gl_grid = gl.GLGridItem()
        gl_grid.scale(0.01*self.unit_scale, 0.01*self.unit_scale, 0.01*self.unit_scale)
        self.gl_widget.addItem(gl_grid)
        # Add Geometry Item
        # self.add_geometry_view(gl_widget)
        (values_norm, color_values_norm) = self.image_filter.color_normalizer(value=value,
                                                               cmap=cmap,
                                                               base_opacity=base_opacity,
                                                               norm_method=norm_method,
                                                               gamma=gamma,
                                                               initial=True,
                                                               coordinates=self.sensing_domain_coordinates)
        # Set opacity filter
        color_values_final = self.image_filter.opacity_filter(opacity_filter=opacity_filter,
                                                              opacity_param=opacity_param,
                                                              values=value,
                                                              values_norm=values_norm,
                                                              color_values_norm=color_values_norm,
                                                              coordinates=self.sensing_domain_coordinates)
        #
        self.item_dof = gl.GLScatterPlotItem(pos=coordinates, color=color_values_final, size=0.001*self.unit_scale, pxMode=False)
        self.item_dof.setGLOptions('additive')
        self.gl_widget.addItem(self.item_dof)

    def debug_tracker(self):
        self.max_epsilon = self.epsilon_sensing_domain.max()
        self.max_epsilon_position = self.sensing_domain_coordinates[self.epsilon_sensing_domain.argmax()]
        print('-------------------------------------------------------')
        print('-------------------------------------------------------')
        print('Max Epsilon = {:.5}'.format(self.max_epsilon))
        print('-------------------------------------------------------')
        print('[Position of Max Epsilon]')
        print('x = {:7.4} mm'.format(self.max_epsilon_position[0] * 1000))
        print('y = {:7.4} mm'.format(self.max_epsilon_position[1] * 1000))
        print('z = {:7.4} mm'.format(self.max_epsilon_position[2] * 1000))
        print('-------------------------------------------------------')
        print('-------------------------------------------------------')

    def debug_plot_phi(self, elec_num, cmap='hot', base_opacity=1.0, norm_method='power', flag_dof_coordinate=True):
        self.solve_phi(elec_num)
        """
        # Choose coordinate option - Degree of freedom (nodal_values) / vertex (vertex_values)
        # For a finite element function from a standard continuous piecewise linear function space (P1 Lagrange elements),
        # Both methods will return the same results, but with possibly different ordering
        # The array vertex_values will have the same ordering as the vertices of the mesh,
        # while nodal_values will be ordered in a way that (nearly) minimizes the bandwidth of the system matrix
        # and thus improves the efficiency of linear solvers.
        """
        if flag_dof_coordinate:
            coordinates = self.phi.function_space().tabulate_dof_coordinates()
            dof = self.phi.vector()[:]
        else:
            coordinates = np.array(
                [self.mesh.coordinates()[:, i] for i in range(self.mesh.geometry().dim())]).transpose()
            dof = self.phi.compute_vertex_values(self.mesh)
        # Plot solution
        self.debug_solver_viewer(coordinates,
                                 dof,
                                 cmap=cmap,
                                 norm_method=norm_method,
                                 gamma=0.5,
                                 base_opacity=base_opacity)

    def debug_plot_sensitivity_map(self, i, j, cmap='seismic', base_opacity=0.01, norm_method='lognormal'):
        grad_phi_i = self.solve_grad_phi(i)
        grad_phi_j = self.solve_grad_phi(j)
        inner = grad_phi_i * grad_phi_j
        sensitivity = (1.0/self.V_exc) * self.cell_volumes * np.sum(inner, axis=0)
        self.debug_solver_viewer(self.cell_coordinates,
                                 sensitivity,
                                 cmap=cmap,
                                 norm_method=norm_method,
                                 gamma=0.5,
                                 base_opacity=base_opacity)

    def debug_plot_forward_cap_array(self):
        import matplotlib.pyplot as plt
        C_forward = self.calculate_forward_cap_array()
        print(C_forward[:])
        plt.plot(C_forward)
        plt.show()

    """
    def add_geometry_view(self, gl_widget):
        # Generate fake color map for geometry object view
        geo = np.ones(self.face_array.shape) * 0.5
        color_map_geo = cmx.get_cmap('gray')
        colors_geo = color_map_geo(geo, alpha=0.1)
        mesh_data = gl.MeshData(vertexes=self.mesh.coordinates(), faces=self.face_array, faceColors=colors_geo)
        item_geo = gl.GLMeshItem(meshdata=mesh_data, drawFaces=True, smooth=True, shader='balloon', glOptions='translucent', computeNormals=False)
        gl_widget.addItem(item_geo)
    """




















