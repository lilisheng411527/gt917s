from building import *
Import('rtconfig')

src   = []
cwd   = GetCurrentDir()

# add gt917s src files.
if GetDepend('PKG_USING_GT917S'):
    src += Glob('src/gt917s.c')

if GetDepend('PKG_USING_GT917S_SAMPLE'):
    src += Glob('samples/gt917s_sample.c')

# add gt917s include path.
path  = [cwd + '/inc']

# add src and include to group.
group = DefineGroup('gt917s', src, depend = ['PKG_USING_GT917S'], CPPPATH = path)

Return('group')
