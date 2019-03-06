def markers():
    # 3D domain marker dictionary
    subdomain_dict = {
        'air': 10,
        'pcb': 11,
    }
    # 2D facet marker dictionary
    boundary_dict = {
        'elec1': 1,
        'elec2': 2,
        'elec3': 3,
        'elec4': 4,
        'elec5': 5,
        'elec6': 6,
        'elec7': 7,
        'elec8': 8,
        'gnd_cu': 9,
        'air': 10,
    }
    return (subdomain_dict, boundary_dict)

def materials(subdomain_dict):
    k_dict = {
        subdomain_dict.get('air'): 1.0,
        subdomain_dict.get('pcb'): 4.5,
    }
    return (k_dict,)