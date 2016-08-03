"""A replacement transport for Python xmlrpc library."""

try:
    import xmlrpc.client as xmlrpc
except ImportError:
    import xmlrpclib as xmlrpc

import socket
import requests
import urllib3.exceptions


class RequestsTransport(xmlrpc.Transport):
    """Drop in Transport for xmlrpclib that uses Requests instead of httplib"""

    user_agent = "Python XMLRPC with Requests (python-requests.org)"

    def __init__(self, scheme, use_datetime=0):
        xmlrpc.Transport.__init__(self, use_datetime)
        self._scheme = scheme

    def request(self, host, handler, request_body, verbose=0):
        """Make a xmlrpc request."""
        headers = {'User-Agent': self.user_agent, 'Content-Type': 'text/xml'}
        url = '{scheme}://{host}{handler}'.format(scheme=self._scheme,
                                                   host=host,
                                                   handler=handler)
        try:
            resp = requests.post(url, data=request_body.encode('utf-8'),
                                 headers=headers)
        except requests.exceptions.Timeout:
            raise socket.timeout('timed out')
        except requests.RequestException as exc:
            if isinstance(exc.args[0], urllib3.exceptions.HTTPError):
                # Rethrow urllib3 exception.reason. That are e.g. socket.error
                # Or other exceptions that the default xmlrpclib implementation
                # would throw.
                raise exc.args[0].reason
            else:
                # otherwise, rethrow the exc
                # We could add more exception mappings here:
                #  - requests.exceptions.InvalidURL    -> socket.gaierror?
                #  - requests.exceptions.InvalidSchema -> socket.gaierror?
                raise exc
        except (ValueError, Exception):
            raise
        else:
            try:
                resp.raise_for_status()
            except requests.RequestException as exc:
                raise xmlrpc.ProtocolError(url, resp.status_code,
                                           str(exc), resp.headers)
            else:
                return self.parse_response(resp)

    def parse_response(self, resp):
        """
        Parse the xmlrpc response.
        """
        p, u = self.getparser()
        p.feed(resp.text)
        p.close()
        return u.close()
