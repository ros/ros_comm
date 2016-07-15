ROLE_STRUCT = {
    'topics':{
        'subscriber':{
            'mask':'s', # subscribe
            'OID':['CLIENT_AUTH', '1.3.6.1.5.5.7.3.4'], # emailProtection
        },
        'publisher':{
            'mask':'p', # publish
            'OID':['SERVER_AUTH', '1.3.6.1.5.5.7.3.3'], # codeSigning
        },
    },
    'services':{
        'client':{
            'mask':'c', # call
            'OID':['CLIENT_AUTH', '1.3.6.1.5.5.7.3.21'], # secureShellClient
        },
        'server':{
            'mask':'x', # execute
            'OID':['SERVER_AUTH', '1.3.6.1.5.5.7.3.22'], # secureShellServer
        },
    },
    'parameters':{
        'reader':{
            'mask':'r', # read
            'OID':['CLIENT_AUTH', '1.3.6.1.5.5.7.3.16'], # id-kp-scvpClient
        },
        'writer':{
            'mask':'w', # write
            'OID':['CLIENT_AUTH', '1.3.6.1.5.5.7.3.15'], # id-kp-scvpServer
        },
    }
}

EXTENSION_MAPPING = {
    'key': '.pem',
    'cert': '.cert',
}