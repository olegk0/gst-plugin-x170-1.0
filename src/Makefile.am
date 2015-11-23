# plugindir is set in configure

##############################################################################
# change libgstplugin.la to something more suitable, e.g. libmysomething.la  #
##############################################################################
plugin_LTLIBRARIES = libgstx170.la

AM_CPPFLAGS = @GST_BASE_CFLAGS@

##############################################################################
# for the next set of variables, rename the prefix if you renamed the .la,   #
#  e.g. libgstplugin_la_SOURCES => libmysomething_la_SOURCES                 #
#       libgstplugin_la_CFLAGS  => libmysomething_la_CFLAGS                  #
#       libgstplugin_la_LIBADD  => libmysomething_la_LIBADD                  #
#       libgstplugin_la_LDFLAGS => libmysomething_la_LDFLAGS                 #
##############################################################################

# sources used to compile this plug-in
libgstx170_la_SOURCES = gstx170.c

# flags used to compile this plugin
# add other _CFLAGS and _LIBS as needed
libgstx170_la_CFLAGS = -Wall -Wstrict-prototypes
libgstx170_la_LDFLAGS = $(GST_PLUGIN_LDFLAGS) -ldecx170h -ldecx170m2 -ldecx170m -ldecx170p -ldecx170v -ldwlx170 -lx170j

# headers we need but don't want installed
noinst_HEADERS = gstx170.h
