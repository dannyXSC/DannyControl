import os
import re
from abc import ABC, abstractmethod


class FileNameGenerator:
    def __init__(self, folder, template):
        """
        :param folder: 文件夹路径
        :param template: 文件名模板，格式如 'episode_{}.hdf5'
        """
        self.folder = folder
        self.template = template
        self.pattern = self._generate_pattern(template)
        self.max_index = None

    def _generate_pattern(self, template):
        """
        根据模板生成正则表达式模式，以匹配文件名中的index
        """
        escaped_template = re.escape(template)
        return re.compile(escaped_template.replace(r'\{', r'(\d+)').replace(r'\}', r''))
    
    def _extract_index(self, filename):
        """
        从文件名中提取index
        """
        match = self.pattern.match(filename)
        if match:
            return int(match.group(1))
        return None

    def get_next_file_name(self):
        """
        找到当前文件夹下index最大的文件，返回下一个文件名（index+1）
        """
        if self.max_index is None:
            max_index = -1
            
            # 遍历文件夹中的所有文件
            for filename in os.listdir(self.folder):
                # 提取文件名中的index
                index = self._extract_index(filename)
                if index is not None and index > max_index:
                    max_index = index

            # 生成下一个文件名
            self.max_index = max_index + 1
        next_file_name = self.template.format(self.max_index)
        self.max_index += 1
        return next_file_name
    
class H5pyDumper(ABC):
    def __init__(self,folder,template) -> None:
        self.folder = folder
        self.file_name_gen = FileNameGenerator(folder,template)
        
        self.init()

    def get_file_name(self):
        return self.file_name_gen.get_next_file_name()

    def init(self):
        self.data_dict = {}

    def empty(self):
        return len(self.data_dict) == 0

    def _dict_append(self, target, source):
        # target source都是字典
        for key, value in source.items():
            if not isinstance(value, dict):
                if key not in target:
                    target[key] = [source[key]]
                else:
                    target[key].append(source[key])
            else:
                if key not in target:
                    target[key] = {}
                self._dict_append(target[key], source[key])

    def add(self, payload):
        # keys in payload must be same all the time
        self._dict_append(self.data_dict, payload)

    @abstractmethod
    def save(self, metadata=None):
        pass