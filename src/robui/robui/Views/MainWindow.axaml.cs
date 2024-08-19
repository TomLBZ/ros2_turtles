using Avalonia.Controls;
using Avalonia.Markup.Xaml;
using robui.ViewModels;

//namespace robui.Views;

namespace robui.Views
{
    public partial class MainWindow : Window
    {
        public MainWindow()
        {
            InitializeComponent();
            DataContext = new MainViewModel();
        }

        private void InitializeComponent()
        {
            AvaloniaXamlLoader.Load(this);
        }
    }
}