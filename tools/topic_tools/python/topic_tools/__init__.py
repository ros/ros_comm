import rospy


__all__ = ('LazyTransport',)


# define a new metaclass which overrides the '__call__' function
# See: http://martyalchin.com/2008/jan/10/simple-plugin-framework/
class MetaLazyTransport(type):

    def __call__(cls, *args, **kwargs):
        """Called when you call LazyTransport()"""
        obj = type.__call__(cls, *args, **kwargs)
        obj._post_init()
        return obj


class LazyTransport(rospy.SubscribeListener):
    __metaclass__ = MetaLazyTransport

    def __init__(self):
        super(LazyTransport, self).__init__()
        self._publishers = []
        # self._connection_status has 3 meanings
        # - None: never been subscribed
        # - False: currently not subscribed but has been subscribed before
        # - True: currently subscribed
        self._connection_status = None
        rospy.Timer(rospy.Duration(5),
                    self._warn_never_subscribed_cb, oneshot=True)

    def _post_init(self):
        if not rospy.get_param('~lazy', True):
            self.subscribe()
            self._connection_status = True

    def _warn_never_subscribed_cb(self, timer_event):
        if self._connection_status is None:
            rospy.logwarn(
                '[{name}] subscribes topics only with'
                " child subscribers. Set '~lazy' as False"
                ' to have it always transport message.'
                .format(name=rospy.get_name()))

    def subscribe(self):
        raise NotImplementedError('Please overwrite this method')

    def unsubscribe(self):
        raise NotImplementedError('Please overwrite this method')

    def peer_subscribe(self, *args, **kwargs):
        rospy.logdebug('[{topic}] is subscribed'.format(topic=args[0]))
        if self._connection_status is not True:
            self.subscribe()
            self._connection_status = True

    def peer_unsubscribe(self, *args, **kwargs):
        rospy.logdebug('[{topic}] is unsubscribed'.format(topic=args[0]))
        if not rospy.get_param('~lazy', True):
            return  # do not unsubscribe
        if self._connection_status in [None, False]:
            return  # no need to unsubscribe
        for pub in self._publishers:
            if pub.get_num_connections() > 0:
                break
        else:
            self.unsubscribe()
            self._connection_status = False

    def advertise(self, *args, **kwargs):
        # subscriber_listener should be 'self'
        # to detect connection and disconnection of the publishing topics
        assert len(args) < 3 or args[2] is None
        assert kwargs.get('subscriber_listener') is None
        kwargs['subscriber_listener'] = self

        pub = rospy.Publisher(*args, **kwargs)
        self._publishers.append(pub)
        return pub
