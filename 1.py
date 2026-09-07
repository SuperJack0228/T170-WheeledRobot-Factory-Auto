import sys
import tty
import termios
import time
import os

class MinimalTextLoader:
    def __init__(self, filepath):
        """
        初始化文本加载器
        
        参数:
            filepath: 文本文件路径
        """
        self.filepath = filepath
        self.text_content = ""
        self.current_position = 0
        self.search_results = []
        self.current_search_index = 0
        self.last_keyword = ""
        
        # 加载文本文件
        self.load_text_file()
    
    def load_text_file(self):
        """加载文本文件"""
        try:
            with open(self.filepath, 'r', encoding='utf-8') as f:
                self.text_content = f.read()
        except Exception as e:
            print(f"错误: 无法读取文件 - {e}")
            sys.exit(1)
    
    def search_text(self, keyword):
        """在文本中搜索关键词"""
        if not keyword:
            return []
        
        self.last_keyword = keyword
        self.search_results = []
        pos = 0
        
        while pos < len(self.text_content):
            found_pos = self.text_content.find(keyword, pos)
            if found_pos == -1:
                break
            
            self.search_results.append(found_pos)
            pos = found_pos + 1
        
        return self.search_results
    
    def get_search_input(self):
        """在原始模式下获取搜索输入"""
        search_term = ""
        
        # 临时显示提示
        os.system('cls' if os.name == 'nt' else 'clear')
        print("搜索: " + search_term, end='', flush=True)
        
        while True:
            ch = sys.stdin.read(1)
            
            # 回车键确认搜索
            if ch == '\r' or ch == '\n':
                break
            # 退格键删除字符
            elif ch == '\x7f' or ch == '\x08':  # Backspace
                if search_term:
                    search_term = search_term[:-1]
            # Ctrl+C 或 Esc 取消
            elif ch == '\x03' or ch == '\x1b':  # Ctrl+C 或 Esc
                return None
            # 普通字符
            elif ch.isprintable():
                search_term += ch
            
            # 更新显示
            os.system('cls' if os.name == 'nt' else 'clear')
            print("搜索: " + search_term, end='', flush=True)
        
        return search_term
    
    def show_30_chars(self, start_pos=None):
        """
        显示30个字符
        
        参数:
            start_pos: 起始位置，如果为None则从当前位置开始
        """
        if not self.text_content:
            return
        
        text_len = len(self.text_content)
        
        if start_pos is not None:
            self.current_position = start_pos
        
        # 确保位置不越界
        if self.current_position >= text_len:
            self.current_position = 0
        
        # 获取30个字符
        end_pos = min(text_len, self.current_position + 30)
        text_to_show = self.text_content[self.current_position:end_pos]
        
        # 清空屏幕
        os.system('cls' if os.name == 'nt' else 'clear')
        
        # 只显示30个字
        print(text_to_show)
        
        # 更新位置
        self.current_position = end_pos
    
    def show_with_search_highlight(self, position):
        """显示指定位置的文本，并高亮搜索内容"""
        if not self.text_content or not self.search_results:
            return self.show_30_chars(position)
        
        text_len = len(self.text_content)
        start_pos = max(0, position)
        end_pos = min(text_len, start_pos + 30)
        
        # 获取当前要显示的文本
        text_to_show = self.text_content[start_pos:end_pos]
        
        # 检查当前显示范围内是否有搜索匹配
        current_match = None
        for match_pos in self.search_results:
            if start_pos <= match_pos < end_pos:
                current_match = match_pos
                break
        
        # 清空屏幕
        os.system('cls' if os.name == 'nt' else 'clear')
        
        if current_match is not None and self.last_keyword:
            # 计算在当页中的位置
            match_in_page = current_match - start_pos
            keyword_len = len(self.last_keyword)
            
            # 分割文本以高亮显示
            before_match = text_to_show[:match_in_page]
            match_text = text_to_show[match_in_page:match_in_page + keyword_len]
            after_match = text_to_show[match_in_page + keyword_len:]
            
            # 使用ANSI转义码高亮显示
            highlighted_text = f"{before_match}\033[91m{match_text}\033[0m{after_match}"
            print(highlighted_text)
        else:
            print(text_to_show)
        
        # 更新位置
        self.current_position = end_pos
    
    def handle_search(self):
        """处理搜索"""
        # 获取搜索词
        search_term = self.get_search_input()
        
        if search_term is None:  # 用户取消
            # 返回之前的位置
            self.show_30_chars(self.current_position - 30)
            return
        
        if search_term:
            # 搜索文本
            self.search_text(search_term)
            
            if self.search_results:
                # 跳转到第一个匹配
                self.current_search_index = 0
                first_match = self.search_results[0]
                # 显示匹配位置附近的30个字
                show_pos = max(0, first_match - 10)
                self.show_with_search_highlight(show_pos)
            else:
                # 没有找到，从当前位置开始
                self.show_30_chars()
        else:
            # 搜索词为空，从当前位置开始
            self.show_30_chars()
    
    def navigate_to_next_match(self):
        """导航到下一个匹配"""
        if not self.search_results:
            return
        
        if self.current_search_index < len(self.search_results) - 1:
            self.current_search_index += 1
        else:
            # 循环到第一个
            self.current_search_index = 0
        
        match_pos = self.search_results[self.current_search_index]
        # 显示匹配位置附近的30个字
        show_pos = max(0, match_pos - 10)
        self.show_with_search_highlight(show_pos)
    
    def navigate_to_prev_match(self):
        """导航到上一个匹配"""
        if not self.search_results:
            return
        
        if self.current_search_index > 0:
            self.current_search_index -= 1
        else:
            # 循环到最后一个
            self.current_search_index = len(self.search_results) - 1
        
        match_pos = self.search_results[self.current_search_index]
        # 显示匹配位置附近的30个字
        show_pos = max(0, match_pos - 10)
        self.show_with_search_highlight(show_pos)
    
    def run(self):
        """运行主程序"""
        # 保存终端设置
        old_settings = termios.tcgetattr(sys.stdin)
        
        try:
            tty.setraw(sys.stdin.fileno())
            
            # 初始显示前30个字
            self.show_30_chars(0)
            
            while True:
                # 读取单个字符
                ch = sys.stdin.read(1)
                
                if ch == 'q' or ch == 'Q':
                    # 恢复终端设置
                    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old_settings)
                    # 清空屏幕后退出
                    os.system('cls' if os.name == 'nt' else 'clear')
                    break
                
                elif ch == 'a' or ch == 'A':
                    # 显示下一页30个字
                    if self.search_results:
                        self.show_with_search_highlight(self.current_position)
                    else:
                        self.show_30_chars()
                
                elif ch == 's' or ch == 'S':
                    self.handle_search()
                
                elif ch == 'n' or ch == 'N':
                    self.navigate_to_next_match()
                
                elif ch == 'p' or ch == 'P':
                    self.navigate_to_prev_match()
                
                # 小延迟
                time.sleep(0.01)
        
        except KeyboardInterrupt:
            pass
        finally:
            # 确保恢复终端设置
            try:
                termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old_settings)
                os.system('cls' if os.name == 'nt' else 'clear')
            except:
                pass


def main():
    """主函数"""
    if len(sys.argv) < 2:
        print("使用方法: python text_reader.py <文本文件路径>")
        sys.exit(1)
    
    filepath = sys.argv[1]
    
    # 检查文件是否存在
    if not os.path.exists(filepath):
        print(f"错误: 文件 '{filepath}' 不存在")
        sys.exit(1)
    
    # 创建并运行文本加载器
    try:
        loader = MinimalTextLoader(filepath)
        loader.run()
    except Exception as e:
        print(f"错误: {e}")


if __name__ == "__main__":
    main()