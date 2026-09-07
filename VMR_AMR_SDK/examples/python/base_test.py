import py_client_common as rpc_common
import lift as lift

config = rpc_common.ClientConfig()

config_dict = rpc_common.get_client_config()
print(config_dict)