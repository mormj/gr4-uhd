gen_grc_source = custom_target('gen_grc',
                        output : ['uhd_usrp_source.block.yml'
                        ], 
                        command : ['python3', join_paths(meson.current_source_dir(),'gen_uhd_usrp_blocks.py'),
                            '@OUTPUT@'],
                        install : true,
                        install_dir : ['share/gnuradio/grc/blocks'])

install_data(
    'uhd.tree.yml',
    install_dir : 'share/gnuradio/grc/blocks'
)