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
    'certificate': '.cert',
}

# MODE_POLICY_TYPE_MAPPING = {
#     'topic_subscriber': 'topics',
#     'topic_publisher':  'topics',
#     'service_client':   'services',
#     'service_server':   'services',
#     'parameter_reader': 'parameters',
#     'parameter_writer': 'parameters',
# }
#
# MODE_MASK_MAPPING = {
#     'topic.subscriber': 's', # subscribe
#     'topic.publisher':  'p', # publish
#     'service.client':   'c', # call
#     'service.server':   'x', # execute
#     'parameter.reader': 'r', # read
#     'parameter.writer': 'w', # write
# }
# MODE_KEY_USAGE_MAPPING = {
#     'topic.subscriber': 'CLIENT_AUTH',
#     'topic.publisher':  'SERVER_AUTH',
#     'service.client':   'TIME_STAMPING',
#     'service.server':   'CODE_SIGNING',
#     'parameter.reader': '1.3.6.1.5.5.7.3.21', # secureShellClient
#     'parameter.writer': '1.3.6.1.5.5.7.3.22', # secureShellServer
# }