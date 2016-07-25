# Current OIDs are tentative and and will change after we finalizes/register a proper parent OID subtree
ROLE_STRUCT = {
    'topics':{
        'subscriber':{
            'mask':'s', # subscribe
            'OID':'1.2.3.4.5.6.7.8.9.1', 
            'allow':{'OID':'1.2.3.4.5.6.7.8.9.1.1'},
            'deny' :{'OID':'1.2.3.4.5.6.7.8.9.1.2'},
        },
        'publisher':{
            'mask':'p', # publish
            'OID':'1.2.3.4.5.6.7.8.9.2', 
            'allow':{'OID':'1.2.3.4.5.6.7.8.9.2.1'},
            'deny' :{'OID':'1.2.3.4.5.6.7.8.9.2.2'},
        },
    },
    'services':{
        'client':{
            'mask':'c', # call
            'OID':'1.2.3.4.5.6.7.8.9.3', 
            'allow':{'OID':'1.2.3.4.5.6.7.8.9.3.1'},
            'deny' :{'OID':'1.2.3.4.5.6.7.8.9.3.2'},
        },
        'server':{
            'mask':'x', # execute
            'OID':'1.2.3.4.5.6.7.8.9.4', 
            'allow':{'OID':'1.2.3.4.5.6.7.8.9.4.1'},
            'deny' :{'OID':'1.2.3.4.5.6.7.8.9.4.2'},
        },
    },
    'parameters':{
        'reader':{
            'mask':'r', # read
            'OID':'1.2.3.4.5.6.7.8.9.5', 
            'allow':{'OID':'1.2.3.4.5.6.7.8.9.5.1'},
            'deny' :{'OID':'1.2.3.4.5.6.7.8.9.5.2'},
        },
        'writer':{
            'mask':'w', # write
            'OID':'1.2.3.4.5.6.7.8.9.6', 
            'allow':{'OID':'1.2.3.4.5.6.7.8.9.6.1'},
            'deny' :{'OID':'1.2.3.4.5.6.7.8.9.6.2'},
        },
    },
}

EXTENSION_MAPPING = {
    'key': '.pem',
    'cert': '.cert',
}