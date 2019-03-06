from dolfin import *


def xml_to_hdf5(mesh_dir, mesh_filename, subdomain, boundary):
    xml_mesh = Mesh(mesh_dir + mesh_filename + '.xml')
    hdf5_filename = mesh_filename + '.h5'

    hdf = HDF5File(xml_mesh.mpi_comm(), mesh_dir + hdf5_filename, 'w')
    hdf.write(xml_mesh, '/mesh')

    if subdomain:
        xml_subdomain_filename = mesh_filename + '_physical_region.xml'
        xml_subdomains = MeshFunction('size_t', xml_mesh, mesh_dir + xml_subdomain_filename)
        hdf.write(xml_subdomains, '/subdomains')

    if boundary:
        xml_boundary_filename = mesh_filename + '_facet_region.xml'
        xml_boundaries = MeshFunction('size_t', xml_mesh, mesh_dir + xml_boundary_filename)
        hdf.write(xml_boundaries, '/boundaries')
    return hdf5_filename


def read_xml_mesh(mesh_dir, mesh_filename, subdomain, boundary):
    mesh = Mesh(mesh_dir + mesh_filename + '.xml')
    subdomains = None
    boundaries = None
    # If there are subdomains, get subdomain markers
    if subdomain:
        xml_subdomain_filename = mesh_filename + '_physical_region.xml'
        subdomains = MeshFunction('size_t', mesh, mesh_dir + xml_subdomain_filename)
    # If there are boundires, get boundary markers
    if boundary:
        xml_boundary_filename = mesh_filename + '_facet_region.xml'
        boundaries = MeshFunction('size_t', mesh, mesh_dir + xml_boundary_filename)
    return (mesh, subdomains, boundaries)


def read_hdf5_mesh(mesh_dir, mesh_filename, subdomain, boundary):
    mesh = Mesh()
    subdomains = None
    boundaries = None

    # Read mesh data from HDF5 file
    hdf = HDF5File(mesh.mpi_comm(), mesh_dir + mesh_filename + '.h5', 'r')
    hdf.read(mesh, '/mesh', False)

    # If there are subdomains, get subdomain markers
    if subdomain:
        subdomains = MeshFunction('size_t', mesh, mesh.topology().dim())
        hdf.read(subdomains, '/subdomains')

    # If there are boundires, get boundary markers
    if boundary:
        boundaries = MeshFunction('size_t', mesh, mesh.topology().dim() - 1)
        hdf.read(boundaries, '/boundaries')

    return (mesh, subdomains, boundaries)