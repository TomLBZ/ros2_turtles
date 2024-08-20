using System.Net.WebSockets;
using System.Threading;
using Timer = System.Timers.Timer;
using ElapsedEventArgs = System.Timers.ElapsedEventArgs;
using System;
using System.Text;
using System.Threading.Tasks;

namespace robui.Networking;

internal enum ConnectionState
{
    Connected,
    Disconnected
}

internal class WsClient
{
    private string address;
    private int port;
    private Timer timer;
    private ClientWebSocket socket;
    private CancellationTokenSource cts;
    public bool IsAutoConnect { get; set; } = true;
    public event EventHandler<string>? MessageReceived;
    public event EventHandler<ConnectionState>? ConnectionChanged;
    public WsClient(string address, int port, int recon_freq_sec = 1)
    {
        this.address = address;
        this.port = port;
        double interval = 1.0 / recon_freq_sec;
        timer = new Timer(interval * 1000);
        timer.Elapsed += TimerElapsed;
        socket = new ClientWebSocket();
        cts = new CancellationTokenSource();
        timer.Start();  
    }

    internal void TimerElapsed(object? sender, ElapsedEventArgs e)
    {
        if (IsAutoConnect && socket.State != WebSocketState.Open)
        {
            Connect();
        }
    }

    internal void Update(string address, int port)
    {
        this.address = address;
        this.port = port;
        if (socket.State == WebSocketState.Open)
        {
            Disconnect();
        }
        Connect();
    }

    internal async void Connect()
    {
        try
        {
            if (socket.State == WebSocketState.Open)
            {
                return;
            }

            Uri uri = new($"ws://{address}:{port}");
            await socket.ConnectAsync(uri, cts.Token);
            OnConnectionChanged(ConnectionState.Connected);
            // receive messages
            _ = Task.Run(ReceiveMessagesAsync);
        }
        catch (Exception)
        {
            OnConnectionChanged(ConnectionState.Disconnected);
        }
    }

    internal void Disconnect()
    {
        try
        {
            if (socket.State == WebSocketState.Open || socket.State == WebSocketState.Connecting)
            {
                cts.Cancel();
                socket.Dispose();
                socket = new ClientWebSocket();
                cts = new CancellationTokenSource();
                OnConnectionChanged(ConnectionState.Disconnected);
            }
        }
        catch (Exception)
        {
            OnConnectionChanged(ConnectionState.Disconnected);
        }
    }
    public async Task SendMessageAsync(string message)
    {
        if (socket.State == WebSocketState.Open)
        {
            byte[] buffer = Encoding.UTF8.GetBytes(message);
            await socket.SendAsync(new ArraySegment<byte>(buffer), WebSocketMessageType.Text, true, cts.Token);
        }
    }

    public async Task ReceiveMessagesAsync()
    {
        var buffer = new byte[1024];
        try
        {
            while (socket.State == WebSocketState.Open)
            {
                WebSocketReceiveResult? result = null;
                try
                {
                    result = await socket.ReceiveAsync(new ArraySegment<byte>(buffer), cts.Token);
                }
                catch (OperationCanceledException)
                {
                    continue; // skip the rest of the loop and receive the next message
                }
                catch (Exception)
                {
                    break; // exit the loop if an error occurs
                }
                if (result != null && result.MessageType == WebSocketMessageType.Text)
                {
                    var message = Encoding.UTF8.GetString(buffer, 0, result.Count);
                    OnMessageReceived(message);

                }
                else if (result?.MessageType == WebSocketMessageType.Close)
                {
                    await socket.CloseAsync(WebSocketCloseStatus.NormalClosure, "Closed by client", cts.Token);
                    OnConnectionChanged(ConnectionState.Disconnected);
                    break;
                }
            }
        }
        finally
        {
            if (socket.State != WebSocketState.Open)
            {
                OnConnectionChanged(ConnectionState.Disconnected);
            }
        }
    }
    protected virtual void OnMessageReceived(string message)
    {
        MessageReceived?.Invoke(this, message);
    }
    protected virtual void OnConnectionChanged(ConnectionState state)
    {
        ConnectionChanged?.Invoke(this, state);
    }
}
