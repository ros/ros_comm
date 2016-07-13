ROLE_STRUCT = {
    'topics':{
        'subscriber':{
            'mask':'s', # subscribe
            'OID':'CLIENT_AUTH',
        },
        'publisher':{
            'mask':'p', # publish
            'OID':'SERVER_AUTH',
        },
    },
    'services':{
        'client':{
            'mask':'c', # call
            'OID':'TIME_STAMPING',
        },
        'server':{
            'mask':'x', # execute
            'OID':'CODE_SIGNING',
        },
    },
    'parameters':{
        'reader':{
            'mask':'r', # read
            'OID':'1.3.6.1.5.5.7.3.21', # secureShellClient
        },
        'writer':{
            'mask':'w', # write
            'OID':'1.3.6.1.5.5.7.3.22', # secureShellServer
        },
    }
}

EXTENSION_MAPPING = {
    'key': '.pem',
    'cert': '.cert',
}