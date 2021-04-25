import jsbsim

def get_properties():
    fdm = jsbsim.FGFDMExec('.',None)
    fdm.load_model('XV-3')
    fdm_props = fdm.get_property_catalog('')


    for key,value in zip(fdm_props.keys(),fdm_props.values()):
        print(f'{key} : {value}')
    