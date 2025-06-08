import tkinter as tk
from tkinter import filedialog, messagebox, ttk
import pandas as pd
import matplotlib.pyplot as plt
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg, NavigationToolbar2Tk
from datetime import datetime
import os
import glob

# --- 配置Matplotlib以支持中文 ---
plt.rcParams['font.sans-serif'] = ['SimHei'] 
plt.rcParams['axes.unicode_minus'] = False

# --- 后端逻辑函数  ---
def find_newest_csv(directory_path, file_pattern="*.csv"):
    search_path = os.path.join(directory_path, file_pattern)
    list_of_files = glob.glob(search_path)
    if not list_of_files:
        return None
    latest_file = max(list_of_files, key=os.path.getmtime)
    return latest_file

def parse_time(time_str):
    try:
        parts = str(time_str).split("::")
        main_part = parts[0]
        millis = parts[1] if len(parts) > 1 else '0'
        adjusted = f"{main_part}.{millis.zfill(3)}"
        return datetime.strptime(adjusted, "%Y/%m/%d %H:%M:%S.%f")
    except Exception:
        return pd.NaT

# --- 主应用程序类 ---
class DataVisualizerApp(tk.Tk):
    def __init__(self):
        super().__init__()
        self.title("CSV 数据交互式可视化工具- RC9")
        self.geometry("1200x800")

        self.df = None
        self.filepath = tk.StringVar(value="尚未选择文件")
        self.time_column = 'RX Date/Time'
        self.column_vars = {}
        self.plot_type = tk.StringVar(value="timeseries")

        control_frame = ttk.Frame(self, width=300, padding="10")
        control_frame.pack(side="left", fill="y", expand=False)
        
        self.plot_frame = ttk.Frame(self)
        self.plot_frame.pack(side="right", fill="both", expand=True)

        self.create_control_widgets(control_frame)

        self.create_plot_canvas()

        self.auto_load_newest_csv()

    def create_control_widgets(self, parent):
        file_frame = ttk.LabelFrame(parent, text="文件操作", padding="10")
        file_frame.pack(fill="x", pady=5)
        
        ttk.Button(file_frame, text="重新选择CSV文件", command=self.select_csv_file).pack(fill="x", pady=2)
        ttk.Label(file_frame, textvariable=self.filepath, wraplength=250).pack(fill="x", pady=2)

        plot_type_frame = ttk.LabelFrame(parent, text="1. 选择图表类型", padding="10")
        plot_type_frame.pack(fill="x", pady=5)
        
        ttk.Radiobutton(plot_type_frame, text="时序图 (可多选)", variable=self.plot_type, value="timeseries").pack(anchor="w")
        ttk.Radiobutton(plot_type_frame, text="XY二维图 (选择偶数个)", variable=self.plot_type, value="xy").pack(anchor="w")

        self.column_frame = ttk.LabelFrame(parent, text="2. 选择数据列", padding="10")
        self.column_frame.pack(fill="both", expand=True, pady=5)
        
        action_frame = ttk.Frame(parent, padding="10")
        action_frame.pack(fill="x", side="bottom")
        ttk.Button(action_frame, text="生成图表", command=self.plot_data, style='Accent.TButton').pack(fill="x")

    def create_plot_canvas(self):
        """创建Matplotlib画布并嵌入到Tkinter窗口"""
        self.fig, self.ax = plt.subplots(figsize=(10, 8))
        self.canvas = FigureCanvasTkAgg(self.fig, master=self.plot_frame)
        
        toolbar = NavigationToolbar2Tk(self.canvas, self.plot_frame)
        toolbar.update()

        toolbar.pack(side=tk.BOTTOM, fill=tk.X)

        self.canvas.get_tk_widget().pack(side=tk.TOP, fill=tk.BOTH, expand=True)

    def auto_load_newest_csv(self):
        csv_directory = "C:\\Users\\Weica\\Documents\\Serial Studio\\CSV\\RC9\\" 
        latest_file = find_newest_csv(csv_directory)
        if latest_file:
            self.load_csv(latest_file)
        else:
            messagebox.showwarning("未找到文件", f"在目录 '{csv_directory}' 中没有找到任何CSV文件。请手动选择一个文件。")

    def select_csv_file(self):
        filepath = filedialog.askopenfilename(
            title="请选择一个CSV文件",
            filetypes=[("CSV Files", "*.csv"), ("All Files", "*.*")]
        )
        if filepath:
            self.load_csv(filepath)

    def load_csv(self, filepath):
        try:
            df = pd.read_csv(filepath, encoding='utf-8-sig')
            
            if self.time_column not in df.columns:
                messagebox.showerror("错误", f"CSV文件中未找到必需的时间列: '{self.time_column}'")
                return
            
            df[self.time_column] = df[self.time_column].apply(parse_time)
            df.dropna(subset=[self.time_column], inplace=True)
            
            if df.empty:
                messagebox.showerror("错误", "处理后数据为空，请检查CSV文件内容和时间格式。")
                return

            self.df = df
            self.filepath.set(os.path.basename(filepath))
            self.update_column_checklist()

        except Exception as e:
            messagebox.showerror("文件读取失败", f"加载或处理文件时发生错误:\n{e}")
            self.df = None
            self.filepath.set("文件加载失败")
            self.update_column_checklist()

    def update_column_checklist(self):
        for widget in self.column_frame.winfo_children():
            widget.destroy()
        self.column_vars.clear()

        if self.df is None:
            ttk.Label(self.column_frame, text="请先加载一个有效的CSV文件").pack()
            return
            
        data_columns = [col for col in self.df.columns if col != self.time_column]

        for col_name in data_columns:
            var = tk.BooleanVar()
            cb = ttk.Checkbutton(self.column_frame, text=col_name, variable=var)
            cb.pack(anchor="w")
            self.column_vars[col_name] = var

    def get_selected_columns(self):
        return [name for name, var in self.column_vars.items() if var.get()]

    def plot_data(self):
        if self.df is None:
            messagebox.showerror("错误", "没有加载任何数据，无法绘图。")
            return

        selected_cols = self.get_selected_columns()
        plot_mode = self.plot_type.get()
        
        if not selected_cols:
            messagebox.showwarning("提示", "请至少选择一个数据列。")
            return

        self.ax.clear()

        try:
            if plot_mode == "timeseries":
                for col_to_plot in selected_cols:
                    plot_df = self.df[[self.time_column, col_to_plot]].dropna()
                    if not plot_df.empty:
                        self.ax.plot(plot_df[self.time_column], plot_df[col_to_plot], 
                                     marker='.', markersize=3, linestyle='-', label=col_to_plot)
                
                self.ax.set_title(f"时序图", fontsize=16)
                self.ax.set_xlabel("时间", fontsize=12)
                self.ax.set_ylabel("数值", fontsize=12)
                self.ax.legend()
                self.fig.autofmt_xdate(rotation=45)

            elif plot_mode == "xy":
                if len(selected_cols) % 2 != 0 or len(selected_cols) == 0:
                    messagebox.showerror("选择错误", "绘制XY二维图，请选择偶数个数据列进行配对。")
                    return
                
                for i in range(0, len(selected_cols), 2):
                    x_col = selected_cols[i]
                    y_col = selected_cols[i+1]
                    
                    plot_df = self.df[[x_col, y_col]].dropna()
                    if not plot_df.empty:
                        self.ax.plot(plot_df[x_col], plot_df[y_col], 
                                     marker='.', markersize=3, linestyle='-', label=f'{y_col} vs {x_col}')

                self.ax.set_title(f"XY二维图", fontsize=16)
                self.ax.set_xlabel("X轴", fontsize=12)
                self.ax.set_ylabel("Y轴", fontsize=12)
                self.ax.axis('equal')
                self.ax.legend()
            
            self.ax.grid(True)
            self.canvas.draw()

        except Exception as e:
            messagebox.showerror("绘图失败", f"绘图时发生错误:\n{e}")

if __name__ == "__main__":
    app = DataVisualizerApp()
    style = ttk.Style(app)
    style.configure('Accent.TButton', font=('Helvetica', 10, 'bold'))
    app.mainloop()