import numpy as np
import os
from dolfin import Cell, MeshEntity

def convert_vector_nodes_to_cells(root_dir, mesh, V):
    # Extract constants
    total_cell_num = mesh.num_cells()
    dofs_num_per_cell = len(V.dofmap().cell_dofs(0))
    # Pre-define empty arrays
    dof_index_to_cell_table = np.empty((total_cell_num, dofs_num_per_cell), dtype=np.uint32)
    cell_volumes = np.empty(total_cell_num)
    cell_coordinates = np.empty((total_cell_num, 3))
    # Generate cell-to-dof-indices & cell-to-volume matching table & cell coordinates
    file_path1 = root_dir + 'dof_index_to_cell_table.npy'
    file_path2 = root_dir + 'cell_volumes.npy'
    file_path3 = root_dir + 'cell_coordinates.npy'
    if (os.path.isfile(file_path1)) & (os.path.isfile(file_path2)) & (os.path.isfile(file_path3)):
        dof_index_to_cell_table = np.load(file_path1)
        cell_volumes = np.load(file_path2)
        cell_coordinates = np.load(file_path3)
    else:
        print("No cell information file detected")
        print(" ==> Creating new cell information")
        for cell_num in range(total_cell_num):
            dof_index_to_cell_table[cell_num] = V.dofmap().cell_dofs(cell_num)
            cell_volumes[cell_num] = Cell(mesh, cell_num).volume()
            cell_coordinates[cell_num] = MeshEntity(mesh, 3, cell_num).midpoint().array()
        np.save(file_path1, dof_index_to_cell_table)
        np.save(file_path2, cell_volumes)
        np.save(file_path3, cell_coordinates)
    return (cell_volumes, cell_coordinates, dof_index_to_cell_table)

def get_air_cell_indices(root_dir, subdomains, subdomain_dict):
    # Create cell index array for 'air' domain
    file_path = root_dir + 'air_cell_indices.npy'
    if os.path.isfile(file_path):
        air_cell_indices = np.load(file_path)
    else:
        print("No air cell information file detected")
        print("  ==> Creating new air cell information")
        air_cell_indices = []
        for cell_num in range(len(subdomains.array())):
            subdomain_no = subdomains.array()[cell_num]
            if subdomain_no == subdomain_dict.get('air'):
                air_cell_indices.append(cell_num)
        air_cell_indices = np.array(air_cell_indices)
        np.save(file_path, air_cell_indices)
    return air_cell_indices

def calculate_face_array(mesh):
    total_face_num = mesh.num_faces()
    face_array = np.empty((total_face_num, 3))
    # Get three vertex indices for each face
    for i in range(total_face_num):
        face_array[i] = MeshEntity(mesh, 2, i).entities(0)
    return face_array
