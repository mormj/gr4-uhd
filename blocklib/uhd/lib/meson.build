uhd_sources += ['usrp_block_impl.cc', 'usrp_source_impl.cc']
uhd_deps += [gnuradio_gr_dep, uhd_dep, volk_dep, fmt_dep, pmtf_dep, python3_embed_dep, json_dep]

block_cpp_args = ['-DHAVE_CPU']

incdir = include_directories(['../include/gnuradio/uhd','../include'])
gnuradio_blocklib_uhd_lib = library('gnuradio-blocklib-uhd', 
    uhd_sources, 
    include_directories : incdir, 
    install : true,
    link_language: 'cpp',
    dependencies : uhd_deps,
    link_args : ['-lgnuradio-runtime'],
    cpp_args : block_cpp_args)

gnuradio_blocklib_uhd_dep = declare_dependency(include_directories : incdir,
					   link_with : gnuradio_blocklib_uhd_lib,
                       dependencies : uhd_deps)
