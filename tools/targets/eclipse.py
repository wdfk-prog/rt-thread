#
# Copyright (c) 2006-2022, RT-Thread Development Team
#
# SPDX-License-Identifier: Apache-2.0
#
# Change Logs:
# Date           Author       Notes
# 2019-03-21     Bernard      the first version
# 2019-04-15     armink       fix project update error
#

import glob
import xml.etree.ElementTree as etree
from xml.etree.ElementTree import SubElement

from . import rt_studio
import sys
import os
import zlib

# Add parent directory to path to import building and utils
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
import building as building_module
from building import *

rtconfig = building_module.rtconfig
from utils import *
from utils import _make_path_relative
from utils import xml_indent

MODULE_VER_NUM = 6

source_pattern = ['*.c', '*.cpp', '*.cxx', '*.cc', '*.s', '*.S', '*.asm','*.cmd']


def OSPath(path):
    import platform

    if type(path) == type('str'):
        if platform.system() == 'Windows':
            return path.replace('/', '\\')
        else:
            return path.replace('\\', '/')
    else:
        if platform.system() == 'Windows':
            return [item.replace('/', '\\') for item in path]
        else:
            return [item.replace('\\', '/') for item in path]


# collect the build source code path and parent path
def CollectPaths(paths):
    all_paths = []

    def ParentPaths(path):
        ret = os.path.dirname(path)
        if ret == path or ret == '':
            return []

        return [ret] + ParentPaths(ret)

    for path in paths:
        # path = os.path.abspath(path)
        path = path.replace('\\', '/')
        all_paths = all_paths + [path] + ParentPaths(path)

    cwd = os.getcwd()
    for path in os.listdir(cwd):
        temp_path = cwd.replace('\\', '/') + '/' + path
        if os.path.isdir(temp_path):
            all_paths = all_paths + [temp_path]

    all_paths = list(set(all_paths))
    return sorted(all_paths)


'''
Collect all of files under paths
'''


def CollectFiles(paths, pattern):
    files = []
    for path in paths:
        if type(pattern) == type(''):
            files = files + glob.glob(path + '/' + pattern)
        else:
            for item in pattern:
                # print('--> %s' % (path + '/' + item))
                files = files + glob.glob(path + '/' + item)

    return sorted(files)


def CollectAllFilesinPath(path, pattern):
    files = []

    for item in pattern:
        files += glob.glob(path + '/' + item)

    list = os.listdir(path)
    if len(list):
        for item in list:
            if item.startswith('.'):
                continue
            if item == 'bsp':
                continue

            if os.path.isdir(os.path.join(path, item)):
                files = files + CollectAllFilesinPath(os.path.join(path, item), pattern)
    return files


'''
Exclude files from infiles
'''


def ExcludeFiles(infiles, files):
    in_files = set([OSPath(file) for file in infiles])
    exl_files = set([OSPath(file) for file in files])

    exl_files = in_files - exl_files

    return exl_files


# caluclate the exclude path for project
def ExcludePaths(rootpath, paths):
    ret = []

    files = os.listdir(OSPath(rootpath))
    for file in files:
        fullname = os.path.join(OSPath(rootpath), file)

        if not os.path.isdir(fullname):
            continue

        # Hidden directories may also contain source files. Keep Eclipse
        # exclusions consistent with the directories selected by SCons.
        if fullname not in paths:
            ret.append(fullname)
        else:
            ret.extend(ExcludePaths(fullname, paths))

    return ret


rtt_path_prefix = '"${workspace_loc://${ProjName}//'


def ConverToRttEclipsePathFormat(path):
    return rtt_path_prefix + path + '}"'


def IsRttEclipsePathFormat(path):
    if path.startswith(rtt_path_prefix):
        return True
    else:
        return False


# all libs added by scons should be ends with five whitespace as a flag
rtt_lib_flag = 5 * " "


def ConverToRttEclipseLibFormat(lib):
    return str(lib) + str(rtt_lib_flag)


def IsRttEclipseLibFormat(path):
    if path.endswith(rtt_lib_flag):
        return True
    else:
        return False


def IsCppProject():
    return GetDepend('RT_USING_CPLUSPLUS')

def _stringify_env_flag_list(value):
    # SCons env flags can be a string, list, or tuple depending on toolchain/bsp.
    # Normalize them to a single whitespace-joined string for Eclipse CDT option values.
    if value is None:
        return ''
    if isinstance(value, (list, tuple)):
        return ' '.join(str(item) for item in value)
    return str(value)

def _strip_linker_script_flag(value):
    # Eclipse/RT-Studio project templates usually set the linker script via a dedicated option
    # (e.g. "linker.scriptfile" or "linker.option.script"). Keep "Other linker flags" clean by
    # removing "-T <script>" from LINKFLAGS to avoid duplicated/contradictory settings.
    items = _stringify_env_flag_list(value).split()
    if '-T' not in items:
        return ' '.join(items)

    filtered = []
    index = 0
    while index < len(items):
        if items[index] == '-T':
            index += 2
            continue
        filtered.append(items[index])
        index += 1
    return ' '.join(filtered)


def _write_text_if_changed(path, content, encoding='utf-8'):
    if os.path.exists(path):
        with open(path, 'r', encoding=encoding, errors='ignore', newline='') as existing:
            if existing.read() == content:
                return False

    with open(path, 'w', encoding=encoding, newline='') as output:
        output.write(content)
    return True


def _serialize_xml(root, xml_declaration, processing_instruction=None):
    xml_indent(root)
    header = xml_declaration + '\n'
    if processing_instruction is not None:
        header += processing_instruction
    body = etree.tostring(root, encoding='utf-8').decode('utf-8')
    return header + body


def _path_is_within(path, root):
    path = os.path.abspath(path)
    root = os.path.abspath(root)
    try:
        common = os.path.commonpath([path, root])
    except ValueError:
        return False
    return os.path.normcase(common) == os.path.normcase(root)


def _path_equal(left, right):
    return os.path.normcase(os.path.abspath(left)) == os.path.normcase(os.path.abspath(right))


def _stable_numeric_id(*parts):
    key = '|'.join(str(part) for part in parts)
    return str(zlib.crc32(key.encode('utf-8')) & 0xffffffff)


def _as_list(value):
    if value is None:
        return []
    if isinstance(value, (list, tuple)):
        return list(value)
    return [value]


def _format_cppdefines(defines):
    result = []
    for item in _as_list(defines):
        if isinstance(item, dict):
            for name, value in item.items():
                result.append(str(name) if value is None else f'{name}={value}')
        elif isinstance(item, (list, tuple)) and len(item) >= 2:
            result.append(str(item[0]) if item[1] is None else f'{item[0]}={item[1]}')
        else:
            result.append(str(item))
    return result


def _append_unique(target, values):
    for value in values:
        if value not in target:
            target.append(value)


def _join_flags(*values):
    parts = []
    for value in values:
        text = _stringify_env_flag_list(value).strip()
        if text:
            parts.append(text)
    return ' '.join(parts)


def _source_node_abspath(source):
    # A SCons source listed from a VariantDir may point at the variant tree
    # (for example, build/kernel/src/clock.c). Eclipse resource paths must
    # refer to the real project/linked source instead, so resolve srcnode().
    srcnode = getattr(source, 'srcnode', None)
    if callable(srcnode):
        source = srcnode()

    source_path = getattr(source, 'abspath', str(source))
    return os.path.abspath(str(source_path))


def _source_tool_kind(source_path):
    extension = os.path.splitext(source_path)[1]
    if extension == '.c':
        return 'c'
    if extension in ('.cpp', '.cxx', '.cc'):
        return 'cpp'
    if extension in ('.s', '.S', '.asm'):
        return 'assembler'
    return None


def _find_tool(tools, kind):
    tool_markers = {
        'c': '.tool.c.compiler',
        'cpp': '.tool.cpp.compiler',
        'assembler': '.tool.assembler',
    }
    marker = tool_markers[kind]
    for tool in tools:
        if marker in (tool.get('id') or ''):
            return tool
    return None


def _find_tool_option(tool, kind, option_kind):
    fragments = {
        ('c', 'defines'): ('compiler.defs', 'compiler.option.definedsymbols'),
        ('cpp', 'defines'): ('compiler.defs', 'compiler.option.definedsymbols'),
        ('assembler', 'defines'): ('assembler.defs', 'assembler.option.definedsymbols'),
        ('c', 'includes'): ('compiler.include.paths', 'compiler.option.includepaths'),
        ('cpp', 'includes'): ('compiler.include.paths', 'compiler.option.includepaths'),
        ('assembler', 'includes'): ('assembler.include.paths', 'assembler.option.includepaths'),
        ('c', 'other'): ('option.c.compiler.other',),
        ('cpp', 'other'): ('option.cpp.compiler.other',),
        ('assembler', 'other'): ('option.assembler.other',),
    }[(kind, option_kind)]

    for option in tool.findall('option'):
        option_id = option.get('id') or ''
        if any(fragment in option_id for fragment in fragments):
            return option
    return None


def _copy_option_attributes(option, excluded=()):
    return {key: value for key, value in option.attrib.items() if key not in excluded}


def _add_file_list_option(tool, root_option, values, suffix):
    if root_option is None or not values:
        return

    attrs = _copy_option_attributes(root_option, excluded=('id', 'value'))
    attrs['id'] = f"{root_option.get('id')}.{suffix}"
    attrs['superClass'] = root_option.get('superClass') or root_option.get('id')
    option = SubElement(tool, 'option', attrs)

    existing_values = []
    for item in root_option.findall('listOptionValue'):
        item_attrs = dict(item.attrib)
        existing_values.append(item_attrs.get('value'))
        SubElement(option, 'listOptionValue', item_attrs)

    for value in values:
        if value in existing_values:
            continue
        SubElement(option, 'listOptionValue', {'builtIn': 'false', 'value': value})


def _add_file_string_option(tool, root_option, value, suffix):
    if root_option is None or not value:
        return

    attrs = _copy_option_attributes(root_option, excluded=('id', 'value'))
    attrs['id'] = f"{root_option.get('id')}.{suffix}"
    attrs['superClass'] = root_option.get('superClass') or root_option.get('id')
    attrs['value'] = value
    SubElement(tool, 'option', attrs)


def _add_file_input_types(tool, root_tool, suffix):
    for root_input in root_tool.findall('inputType'):
        attrs = _copy_option_attributes(root_input, excluded=('id',))
        attrs['id'] = f"{root_input.get('id')}.{suffix}"
        attrs['superClass'] = root_input.get('superClass') or root_input.get('id')
        SubElement(tool, 'inputType', attrs)


def _collect_local_source_settings(env):
    settings = {}
    local_keys = (
        'LOCAL_CFLAGS',
        'LOCAL_CXXFLAGS',
        'LOCAL_CCFLAGS',
        'LOCAL_CPPPATH',
        'LOCAL_CPPDEFINES',
        'LOCAL_ASFLAGS',
    )

    for group in building_module.Projects:
        if not any(group.get(key) for key in local_keys):
            continue

        for source in group.get('src', []):
            source_path = _source_node_abspath(source)
            kind = _source_tool_kind(source_path)
            if kind is None:
                continue

            resource_path = RelativeProjectPath(env, source_path).replace('\\', '/')
            item = settings.setdefault(resource_path, {
                'kind': kind,
                'cppdefines': [],
                'cpppath': [],
                'cflags': [],
                'cxxflags': [],
                'ccflags': [],
                'asflags': [],
            })

            _append_unique(item['cppdefines'], _format_cppdefines(group.get('LOCAL_CPPDEFINES')))
            for include_path in _as_list(group.get('LOCAL_CPPPATH')):
                eclipse_path = RelativeProjectPath(env, os.path.normpath(str(include_path))).replace('\\', '/')
                eclipse_path = ConverToRttEclipsePathFormat(eclipse_path)
                _append_unique(item['cpppath'], [eclipse_path])

            for key, target in (
                ('LOCAL_CFLAGS', 'cflags'),
                ('LOCAL_CXXFLAGS', 'cxxflags'),
                ('LOCAL_CCFLAGS', 'ccflags'),
                ('LOCAL_ASFLAGS', 'asflags'),
            ):
                text = _stringify_env_flag_list(group.get(key)).strip()
                if text:
                    _append_unique(item[target], [text])

    return settings


def _remove_generated_file_info(configuration):
    for file_info in list(configuration.findall('fileInfo')):
        if '.rtthread.scons.' in (file_info.get('id') or ''):
            configuration.remove(file_info)


def _insert_before_source_entries(configuration, element):
    children = list(configuration)
    for index, child in enumerate(children):
        if child.tag == 'sourceEntries':
            configuration.insert(index, element)
            return
    configuration.append(element)


def UpdateLocalSourceSettings(configuration, env):
    _remove_generated_file_info(configuration)

    folder_tools = configuration.findall('folderInfo/toolChain/tool')
    local_settings = _collect_local_source_settings(env)
    if not local_settings:
        return

    config_id = configuration.get('id') or 'rtthread.eclipse'
    for resource_path in sorted(local_settings):
        settings = local_settings[resource_path]
        root_tool = _find_tool(folder_tools, settings['kind'])
        if root_tool is None:
            print(f'WARNING: no Eclipse tool found for local settings: {resource_path}')
            continue

        suffix = _stable_numeric_id(config_id, resource_path, settings['kind'])
        tool_id = f"{root_tool.get('id')}.{suffix}"
        file_info = etree.Element('fileInfo', {
            'id': f'{config_id}.rtthread.scons.{suffix}',
            'name': os.path.basename(resource_path),
            'rcbsApplicability': 'disable',
            'resourcePath': resource_path,
            'toolsToInvoke': tool_id,
        })

        tool_attrs = {
            'id': tool_id,
            'name': root_tool.get('name') or '',
            'superClass': root_tool.get('id'),
        }
        tool = SubElement(file_info, 'tool', tool_attrs)

        include_option = _find_tool_option(root_tool, settings['kind'], 'includes')
        _add_file_list_option(tool, include_option, settings['cpppath'], suffix)

        define_option = _find_tool_option(root_tool, settings['kind'], 'defines')
        _add_file_list_option(tool, define_option, settings['cppdefines'], suffix)

        other_option = _find_tool_option(root_tool, settings['kind'], 'other')
        if settings['kind'] == 'c':
            local_flags = _join_flags(settings['cflags'], settings['ccflags'])
        elif settings['kind'] == 'cpp':
            local_flags = _join_flags(settings['cxxflags'], settings['ccflags'])
        else:
            local_flags = _join_flags(settings['asflags'])
        file_flags = _join_flags(other_option.get('value') if other_option is not None else '', local_flags)
        _add_file_string_option(tool, other_option, file_flags, suffix)

        _add_file_input_types(tool, root_tool, suffix)
        _insert_before_source_entries(configuration, file_info)


def _project_source_folders():
    return list(getattr(rtconfig, 'PROJECT_SOURCE_FOLDERS', []) or [])


def _project_location_uri(project_root, target_path):
    relative_path = os.path.relpath(target_path, project_root).replace('\\', '/')
    if relative_path == '.':
        return 'PROJECT_LOC'

    parts = relative_path.split('/')
    levels_up = 0
    while levels_up < len(parts) and parts[levels_up] == '..':
        levels_up += 1

    tail = '/'.join(parts[levels_up:])
    if levels_up == 0:
        return f'PROJECT_LOC/{tail}'

    prefix = f'PARENT-{levels_up}-PROJECT_LOC'
    return f'{prefix}/{tail}' if tail else prefix


def _resolve_project_location_uri(project_root, location_uri):
    if not location_uri:
        return None
    if location_uri == 'PROJECT_LOC':
        return project_root
    if location_uri.startswith('PROJECT_LOC/'):
        return os.path.abspath(os.path.join(project_root, location_uri[len('PROJECT_LOC/'):]))
    if not location_uri.startswith('PARENT-') or '-PROJECT_LOC' not in location_uri:
        return None

    head, _, tail = location_uri.partition('-PROJECT_LOC')
    try:
        levels_up = int(head[len('PARENT-'):])
    except ValueError:
        return None

    base = project_root
    for _ in range(levels_up):
        base = os.path.dirname(base)
    return os.path.abspath(os.path.join(base, tail.lstrip('/')))


def _desired_project_links(env):
    project_root = os.path.abspath(env['BSP_ROOT'])
    rtt_root = os.path.abspath(env['RTT_ROOT'])
    links = {}

    if not _path_is_within(rtt_root, project_root):
        links['rt-thread'] = rtt_root

    for folder_path in _project_source_folders():
        abs_folder_path = os.path.abspath(os.path.join(project_root, folder_path))
        if _path_equal(abs_folder_path, rtt_root):
            continue

        link_name = os.path.basename(os.path.normpath(folder_path))
        if link_name in links and not _path_equal(links[link_name], abs_folder_path):
            raise ValueError(f'duplicate Eclipse linked resource name: {link_name}')
        links[link_name] = abs_folder_path

    return links


def _remove_shadowing_legacy_links(linked_resources, project_root, desired_links):
    desired_paths = list(desired_links.values())
    desired_names = set(desired_links)

    for link in list(linked_resources.findall('link')):
        name = link.findtext('name')
        if not name or name in desired_names:
            continue

        linked_path = _resolve_project_location_uri(project_root, link.findtext('locationURI'))
        if linked_path is None:
            continue

        descendants = [
            path for path in desired_paths
            if not _path_equal(path, linked_path) and _path_is_within(path, linked_path)
        ]
        if len(descendants) < 2 or _path_is_within(project_root, linked_path):
            continue

        common_parent = os.path.commonpath(descendants)
        if _path_equal(common_parent, linked_path):
            linked_resources.remove(link)
            print(f"Removed obsolete linked resource '{name}' that shadows managed child links.")


def HandleToolOption(tools, env, project, reset):
    BSP_ROOT = os.path.abspath(env['BSP_ROOT'])

    # NOTE:
    # Historically, the .cproject file only got CFLAGS/AFLAGS/LFLAGS injected when it was
    # first created (rt_studio.gen_cproject_file). Subsequent "scons --target=eclipse"
    # updates would not refresh "Other * flags", so changes in rtconfig.py (e.g. -O0/-Os)
    # would not take effect unless the user deleted .cproject manually.
    #
    # Refresh the relevant Eclipse CDT option values every time we update a project.
    asflags = _stringify_env_flag_list(env['ASFLAGS']) if 'ASFLAGS' in env else ''
    cflags = _stringify_env_flag_list(env['CFLAGS']) if 'CFLAGS' in env else ''
    cxxflags = _stringify_env_flag_list(env['CXXFLAGS']) if 'CXXFLAGS' in env else ''
    linkflags = _strip_linker_script_flag(env['LINKFLAGS']) if 'LINKFLAGS' in env else ''

    CPPDEFINES = project['CPPDEFINES']
    paths = [ConverToRttEclipsePathFormat(RelativeProjectPath(env, os.path.normpath(i)).replace('\\', '/')) for i in project['CPPPATH']]

    compile_include_paths_options = []
    assembler_include_paths_options = []
    compile_include_files_options = []
    compile_defs_options = []
    linker_scriptfile_options = []
    linker_script_options = []
    linker_nostart_options = []
    linker_libs_options = []
    linker_paths_options = []

    linker_newlib_nano_options = []

    for tool in tools:

        if tool.get('id').find('compile') != 1:
            options = tool.findall('option')
            # find all compile options
            for option in options:
                option_id = option.get('id')
                if option_id is None:
                    continue
                if option_id.find('option.assembler.other') != -1:
                    # Keep assembler flags in sync with env['ASFLAGS'] (rtconfig.AFLAGS).
                    option.set('value', asflags)
                    continue
                if option_id.find('option.c.compiler.other') != -1:
                    # Keep C compiler flags in sync with env['CFLAGS'] (rtconfig.CFLAGS).
                    option.set('value', cflags)
                    continue
                if option_id.find('option.cpp.compiler.other') != -1:
                    # Keep C++ compiler flags in sync with env['CXXFLAGS'] (rtconfig.CXXFLAGS).
                    option.set('value', cxxflags)
                    continue
                if ('assembler.include.paths' in option_id) or ('assembler.option.includepaths' in option_id):
                    assembler_include_paths_options += [option]
                elif ('compiler.include.paths' in  option_id) or ('compiler.option.includepaths' in  option_id) or ('compiler.tasking.include' in  option_id):
                    compile_include_paths_options += [option]
                elif option.get('id').find('compiler.include.files') != -1 or option.get('id').find('compiler.option.includefiles') != -1 :
                    compile_include_files_options += [option]
                elif option.get('id').find('compiler.defs') != -1 or option.get('id').find('compiler.option.definedsymbols') != -1:
                    compile_defs_options += [option]

        if tool.get('id').find('linker') != -1:
            options = tool.findall('option')
            # find all linker options
            for option in options:
                option_id = option.get('id')
                if option_id is None:
                    continue
                if option_id.find('option.c.linker.other') != -1 or option_id.find('option.cpp.linker.other') != -1:
                    # Keep "Other linker flags" in sync with env['LINKFLAGS'] (rtconfig.LFLAGS),
                    # while avoiding duplicating "-T <script>" which is handled by dedicated options.
                    option.set('value', linkflags)
                    continue
                if option.get('id').find('linker.scriptfile') != -1:
                    linker_scriptfile_options += [option]
                elif option.get('id').find('linker.option.script') != -1:
                    linker_script_options += [option]
                elif option.get('id').find('linker.nostart') != -1:
                    linker_nostart_options += [option]
                elif option.get('id').find('linker.libs') != -1:
                    linker_libs_options += [option]
                elif option.get('id').find('linker.paths') != -1 and 'LIBPATH' in env:
                    linker_paths_options += [option]
                elif option.get('id').find('linker.usenewlibnano') != -1:
                    linker_newlib_nano_options += [option]

    # change the inclue path
    for option in compile_include_paths_options + assembler_include_paths_options:
        # find all of paths in this project
        include_paths = option.findall('listOptionValue')
        for item in include_paths:
            if reset is True or IsRttEclipsePathFormat(item.get('value')) :
                # clean old configuration
                option.remove(item)
        # print('c.compiler.include.paths')
        paths = sorted(paths)
        for item in paths:
            SubElement(option, 'listOptionValue', {'builtIn': 'false', 'value': item})
    # change the inclue files (default) or definitions
    for option in compile_include_files_options:
        # add '_REENT_SMALL' to CPPDEFINES when --specs=nano.specs has select
        if any(option.get('value') == 'true' for option in linker_newlib_nano_options) and '_REENT_SMALL' not in CPPDEFINES:
            CPPDEFINES += ['_REENT_SMALL']

        file_header = '''
#ifndef RTCONFIG_PREINC_H__
#define RTCONFIG_PREINC_H__

/* Automatically generated file; DO NOT EDIT. */
/* RT-Thread pre-include file */

'''
        file_tail = '\n#endif /*RTCONFIG_PREINC_H__*/\n'
        rtt_pre_inc_item = '"${workspace_loc:/${ProjName}/rtconfig_preinc.h}"'
        # save the CPPDEFINES in to rtconfig_preinc.h, but avoid touching the file when
        # the generated content is unchanged. Rewriting this header on every
        # `scons --target=eclipse` run forces dependent sources to rebuild.
        rtconfig_preinc_content = file_header
        for cppdef in CPPDEFINES:
            rtconfig_preinc_content += "#define " + cppdef.replace('=', ' ') + '\n'
        rtconfig_preinc_content += file_tail
        _write_text_if_changed('rtconfig_preinc.h', rtconfig_preinc_content)
        #  change the c.compiler.include.files
        files = option.findall('listOptionValue')
        find_ok = False
        for item in files:
            if item.get('value') == rtt_pre_inc_item:
                find_ok = True
                break
        if find_ok is False:
            SubElement(option, 'listOptionValue', {'builtIn': 'false', 'value': rtt_pre_inc_item})
    if len(compile_include_files_options) == 0:
        for option in compile_defs_options:
            defs = option.findall('listOptionValue')
            project_defs = []
            for item in defs:
                if reset is True:
                    # clean all old configuration
                    option.remove(item)
                else:
                    project_defs += [item.get('value')]
            if len(project_defs) > 0:
                cproject_defs = set(CPPDEFINES) - set(project_defs)
            else:
                cproject_defs = CPPDEFINES

            # print('c.compiler.defs')
            cproject_defs = sorted(cproject_defs)
            for item in cproject_defs:
                SubElement(option, 'listOptionValue', {'builtIn': 'false', 'value': item})

    # update linker script config for both C and C++ linker options
    linker_script = 'link.lds'
    raw_linkflags = _stringify_env_flag_list(env['LINKFLAGS']) if 'LINKFLAGS' in env else ''
    items = raw_linkflags.split(' ')
    if '-T' in items:
        linker_script = items[items.index('-T') + 1]
        linker_script = ConverToRttEclipsePathFormat(linker_script)

    for option in linker_scriptfile_options:
        listOptionValue = option.find('listOptionValue')
        if listOptionValue != None:
            if reset is True or IsRttEclipsePathFormat(listOptionValue.get('value')):
                listOptionValue.set('value', linker_script)
        else:
            SubElement(option, 'listOptionValue', {'builtIn': 'false', 'value': linker_script})
    # scriptfile in stm32cubeIDE
    for option in linker_script_options:
        if '-T' in items:
            linker_script = ConverToRttEclipsePathFormat(items[items.index('-T') + 1]).strip('"')
            option.set('value', linker_script)
    # update nostartfiles config
    for option in linker_nostart_options:
        if raw_linkflags.find('-nostartfiles') != -1:
            option.set('value', 'true')
        else:
            option.set('value', 'false')
    # update libs
    for option in linker_libs_options:
        # remove old libs
        for item in option.findall('listOptionValue'):
            if IsRttEclipseLibFormat(item.get("value")):
                option.remove(item)

        # add new libs
        if 'LIBS' in env:
            for lib in env['LIBS']:
                lib_name = os.path.basename(str(lib))
                if lib_name.endswith('.a'):
                    if lib_name.startswith('lib'):
                        lib = lib_name[3:].split('.')[0]
                    else:
                        lib = ':' + lib_name
                formatedLib = ConverToRttEclipseLibFormat(lib)
                SubElement(option, 'listOptionValue', {
                           'builtIn': 'false', 'value': formatedLib})

    # update lib paths
    for option in linker_paths_options:
        # remove old lib paths
        for item in option.findall('listOptionValue'):
            if IsRttEclipsePathFormat(item.get('value')):
                # clean old configuration
                option.remove(item)
        # add new old lib paths
        for path in env['LIBPATH']:
            SubElement(option, 'listOptionValue', {'builtIn': 'false', 'value': ConverToRttEclipsePathFormat(RelativeProjectPath(env, path).replace('\\', '/'))})

    return

def UpdateProjectStructure(env, prj_name):
    """Keep Eclipse linked resources aligned with RT_ROOT and configured source folders."""
    project_root = os.path.abspath(env['BSP_ROOT'])
    desired_links = _desired_project_links(env)

    if not os.path.exists('.project'):
        print('Error: .project file not found. Cannot update.')
        return

    project_xml = etree.parse('.project')
    root = project_xml.getroot()
    linked_resources = root.find('linkedResources')
    if linked_resources is None:
        linked_resources = SubElement(root, 'linkedResources')

    managed_names = set(desired_links)
    for link in list(linked_resources.findall('link')):
        name = link.findtext('name')
        if name in managed_names:
            linked_resources.remove(link)

    _remove_shadowing_legacy_links(linked_resources, project_root, desired_links)

    for link_name, abs_folder_path in sorted(desired_links.items()):
        print(f"Creating linked resource for '{link_name}' pointing to '{abs_folder_path}'...")
        link_element = SubElement(linked_resources, 'link')
        SubElement(link_element, 'name').text = link_name
        SubElement(link_element, 'type').text = '2'
        SubElement(link_element, 'locationURI').text = _project_location_uri(project_root, abs_folder_path)

    if len(linked_resources) == 0:
        root.remove(linked_resources)

    project_content = _serialize_xml(root, '<?xml version="1.0" encoding="UTF-8"?>')
    _write_text_if_changed('.project', project_content)


def GenExcluding(env, project):
    rtt_root = os.path.abspath(env['RTT_ROOT'])
    bsp_root = os.path.abspath(env['BSP_ROOT'])

    abs_source_folders = []
    try:
        import rtconfig
        if hasattr(rtconfig, 'PROJECT_SOURCE_FOLDERS'):
            for folder in rtconfig.PROJECT_SOURCE_FOLDERS:
                abs_source_folders.append(os.path.abspath(os.path.join(bsp_root, folder)))
    except ImportError:
        pass

    coll_dirs = CollectPaths(project['DIRS'])
    all_paths_temp = [OSPath(path) for path in coll_dirs]
    all_paths = []

    # add used path
    for path in all_paths_temp:
        is_valid_path = False
        # Check whether the path is within BSP, RTT, or any external source folder.
        if _path_is_within(path, bsp_root) or _path_is_within(path, rtt_root):
            is_valid_path = True
        else:
            for source_folder in abs_source_folders:
                if _path_is_within(path, source_folder):
                    is_valid_path = True
                    break
        
        if is_valid_path:
            all_paths.append(path)


    # Exclude the entire unused directory to support external folders.
    exclude_paths = []

    # 1. Exclude unused directories under BSP ROOT
    exclude_paths += ExcludePaths(bsp_root, all_paths)
    
    # 2. Exclude unused directories under RTT ROOT (if it is not within the BSP)
    if not _path_is_within(rtt_root, bsp_root):
        exclude_paths += ExcludePaths(rtt_root, all_paths)
    
    # 3. Exclude all unused directories under external folders.
    for folder in abs_source_folders:
        # Avoid reprocessing folders that have already been processed as RTT_ROOT.
        if not _path_equal(folder, rtt_root):
            exclude_paths += ExcludePaths(folder, all_paths)

    # Filter out the "unused" directories that do not actually have source files.
    filtered_exclude_paths = []
    for path in exclude_paths:
        normalized_path = path.replace('\\', '/')
        if normalized_path.endswith('rt-thread/bsp') or normalized_path.endswith('rt-thread/libcpu'):
            filtered_exclude_paths.append(path)
            continue

        # Hidden directories that are not selected by SCons can be excluded
        # directly. Avoid recursively scanning metadata trees such as .git.
        if os.path.basename(os.path.normpath(path)).startswith('.'):
            filtered_exclude_paths.append(path)
            continue

        if len(CollectAllFilesinPath(path, source_pattern)):
            filtered_exclude_paths.append(path)

    # Convert the path to a project relative path.
    exclude_paths_relative = [RelativeProjectPath(env, path).replace('\\', '/') for path in filtered_exclude_paths]

    # Calculate the individual files that need to be excluded.
    all_files = CollectFiles(all_paths, source_pattern)
    src_files = project['FILES']

    exclude_files = ExcludeFiles(all_files, src_files)
    exclude_files_relative = [RelativeProjectPath(env, file).replace('\\', '/') for file in exclude_files]

    env['ExPaths'] = exclude_paths_relative
    env['ExFiles'] = exclude_files_relative

    return exclude_paths_relative + exclude_files_relative
def RelativeProjectPath(env, path):

    clean_path_str = str(path).strip().strip(',').strip('"')

    try:
        abs_path = os.path.abspath(clean_path_str)
    except Exception:
        return clean_path_str

    project_root = os.path.abspath(env['BSP_ROOT'])

    # 1. Check if the path is within the project root directory (BSP_ROOT)
    if _path_is_within(abs_path, project_root):
        return _make_path_relative(project_root, abs_path)

    # 2. Check if the path is within the RT-Thread root directory.
    rtt_root = os.path.abspath(env['RTT_ROOT'])
    if _path_is_within(abs_path, rtt_root):
        return 'rt-thread/' + _make_path_relative(rtt_root, abs_path)

    # 3. Check the PROJECT_SOURCE_FOLDERS defined in rtconfig.py.
    if hasattr(rtconfig, 'PROJECT_SOURCE_FOLDERS'):
        for folder_entry in rtconfig.PROJECT_SOURCE_FOLDERS:
            # Get the absolute path of the source folder (for example 'E:/.../lib')
            abs_source_folder = os.path.abspath(os.path.join(project_root, folder_entry))

            if _path_is_within(abs_path, abs_source_folder):
                # The link name in the project is the base name of the folder path (for example, '../lib' -> 'lib')
                link_name = os.path.basename(os.path.normpath(folder_entry))

                relative_part = _make_path_relative(abs_source_folder, abs_path)

                return os.path.join(link_name, relative_part).replace('\\', '/')

    print(f'WARNING: The path "{path}" could not be made relative to the project.')
    return clean_path_str


def HandleExcludingOption(entry, sourceEntries, excluding):
    old_excluding = []
    if entry != None:
        exclud = entry.get('excluding')
        if exclud != None:
            old_excluding = entry.get('excluding').split('|')
            sourceEntries.remove(entry)

    value = ''
    for item in old_excluding:
        if item.startswith('//'):
            old_excluding.remove(item)
        else:
            if value == '':
                value = item
            else:
                value += '|' + item

    for item in excluding:
        # add special excluding path prefix for RT-Thread
        item = '//' + item
        if value == '':
            value = item
        else:
            value += '|' + item

    SubElement(sourceEntries, 'entry', {'excluding': value, 'flags': 'VALUE_WORKSPACE_PATH|RESOLVED', 'kind':'sourcePath', 'name':""})

def _ensure_project_nature(root, nature_text, after_text=None):
    natures = root.find('natures')
    if natures is None:
        natures = SubElement(root, 'natures')

    for nature in natures.findall('nature'):
        if nature.text == nature_text:
            return False

    new_nature = etree.Element('nature')
    new_nature.text = nature_text

    insert_index = len(natures)
    if after_text is not None:
        for index, nature in enumerate(list(natures)):
            if nature.text == after_text:
                insert_index = index + 1
                break
    natures.insert(insert_index, new_nature)
    return True


def UpdateProjectName(prj_name):
    """
    Regardless of whether the .project file exists, make sure its name is correct.
    """
    try:
        if not os.path.exists('.project'):
            project_name = prj_name if prj_name else 'rtthread'
            if rt_studio.gen_project_file(os.path.abspath(".project"), project_name, IsCppProject()) is False:
                print('Fail!')
                return
            print("Generated .project file with name:", project_name)

        project_tree = etree.parse('.project')
        root = project_tree.getroot()
        name_element = root.find('name')
        changed = False
        
        if prj_name and name_element is not None and name_element.text != prj_name:
            print(f"Updating project name from '{name_element.text}' to '{prj_name}'...")
            name_element.text = prj_name
            changed = True

        if IsCppProject():
            changed |= _ensure_project_nature(root, 'org.eclipse.cdt.core.ccnature', 'org.eclipse.cdt.core.cnature')

        if changed:
            project_content = _serialize_xml(root, '<?xml version="1.0" encoding="UTF-8"?>')
            _write_text_if_changed('.project', project_content)

    except Exception as e:
        print("Error updating .project file:", e)

def HandleSourceEntries_Global(sourceEntries, excluding):
    """
    Configure the project to include the root folder ("") and exclude all 
    files/folders in the 'excluding' list. This makes all project folders 
    visible in the IDE.
    """
    # To keep the configuration clean, first remove all existing entries
    for entry in sourceEntries.findall('entry'):
        sourceEntries.remove(entry)

    # Join the exclusion list into a single string with the '|' separator
    excluding_str = '|'.join(sorted(excluding))

    # Create a new, single entry for the project root directory
    SubElement(sourceEntries, 'entry', {
        'flags': 'VALUE_WORKSPACE_PATH|RESOLVED',
        'kind': 'sourcePath',
        'name': "",  # An empty string "" represents the project root
        'excluding': excluding_str
    })

def UpdateCproject(env, project, excluding, reset, prj_name):
    excluding = sorted(excluding)

    cproject = etree.parse('.cproject')

    root = cproject.getroot()
    cconfigurations = root.findall('storageModule/cconfiguration')
    for cconfiguration in cconfigurations:
        configuration = cconfiguration.find('storageModule/configuration')
        if configuration is None:
            continue

        tools = configuration.findall('folderInfo/toolChain/tool')
        HandleToolOption(tools, env, project, reset)
        UpdateLocalSourceSettings(configuration, env)

        if prj_name:
            config_element = configuration
            config_element.set('artifactName', prj_name)

            pre_build_step = getattr(rtconfig, 'PRE_BUILD_STEP', None)
            post_build_step = getattr(rtconfig, 'POST_BUILD_STEP', None)

            if pre_build_step:
                config_element.set('prebuildStep', pre_build_step)
                print("Setting/Overwriting Pre-build step...")

            if post_build_step:
                config_element.set('postbuildStep', post_build_step)
                print("Setting/Overwriting Post-build step...")

        sourceEntries = cconfiguration.find('storageModule/configuration/sourceEntries')
        if sourceEntries is not None:
            # Call the new global handler function for source entries
            HandleSourceEntries_Global(sourceEntries, excluding)
            
    # update refreshScope to ensure the project refreshes correctly
    if prj_name:
        prj_name_for_path = '/' + prj_name
        configurations = root.findall('storageModule/configuration')
        for configuration in configurations:
            resource = configuration.find('resource')
            if resource is not None:
                configuration.remove(resource)
            SubElement(configuration, 'resource', {'resourceType': "PROJECT", 'workspacePath': prj_name_for_path})

    # write back to .cproject file only when the generated content changed
    cproject_content = _serialize_xml(
        root,
        '<?xml version="1.0" encoding="UTF-8" standalone="no"?>',
        '<?fileVersion 4.0.0?>'
    )
    _write_text_if_changed('.cproject', cproject_content)

def TargetEclipse(env, reset=False, prj_name=None):
    global source_pattern

    UpdateProjectName(prj_name)

    print('Update eclipse setting...')

    if not os.path.exists('.cproject'):
        if rt_studio.gen_cproject_file(os.path.abspath(".cproject")) is False:
            print('Fail!')
            return

    if not os.path.exists('.project'):
        project_name = prj_name if prj_name else 'rtthread'
        if rt_studio.gen_project_file(os.path.abspath(".project"), project_name, IsCppProject()) is False:
            print('Fail!')
            return

    if not os.path.exists('.settings/projcfg.ini'):
        file = ""
        items = os.listdir(".")
        if len(items) > 0:
            for item in items:
                if item.endswith(".uvprojx") or item.endswith(".uvproj"):
                    file = os.path.abspath(item)
                    break
        chip_name = rt_studio.get_mcu_info(file)
        if rt_studio.gen_projcfg_ini_file(chip_name, prj_name, os.path.abspath(".settings/projcfg.ini")) is False:
            print('Fail!')
            return
    if not os.path.exists('.settings/org.eclipse.core.runtime.prefs'):
        if rt_studio.gen_org_eclipse_core_runtime_prefs(os.path.abspath(".settings/org.eclipse.core.runtime.prefs")) is False:
            print('Fail!')
            return
    if not os.path.exists('makefile.targets'):
        if rt_studio.gen_makefile_targets(os.path.abspath("makefile.targets")) is False:
            print('Fail!')
            return

    project = ProjectInfo(env)

    UpdateProjectStructure(env, prj_name)

    excluding = GenExcluding(env, project)

    UpdateCproject(env, project, excluding, reset, prj_name)

    print('done!')
    return
