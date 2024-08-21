using System.Net.WebSockets;
using System.Threading;
using Timer = System.Timers.Timer;
using ElapsedEventArgs = System.Timers.ElapsedEventArgs;
using System;
using System.Text;
using System.Threading.Tasks;

namespace robui.Networking;

/// <summary>
/// Enum <c>ConnectionState</c> represents the state of the connection.
/// </summary>
internal enum ConnectionState
{
    Connected,
    Disconnected
}

/// <summary>
/// Class <c>WsClient</c> is a simple WebSocket client that provides the following features:
/// It raises events when a message is received or the connection state changes.
/// It provides methods to connect, disconnect, send, and receive messages.
/// </summary>
internal class WsClient
{
    private string address;
    private int port;
    private Timer timer;
    private ClientWebSocket socket;
    private CancellationTokenSource cts;
    public event EventHandler<string>? MessageReceived;
    public event EventHandler<ConnectionState>? ConnectionChanged;
    /// <summary>
    /// Constructor <c>WsClient</c> creates a new WebSocket client.
    /// </summary>
    /// <param name="address">The address of the server</param>
    /// <param name="port">The address of the port</param>
    /// <param name="recon_freq_sec">The reconnection frequency</param>
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

    /// <summary>
    /// The method <c>TimerElapsed</c> is called when the timer elapses.
    /// </summary>
    /// <param name="sender"></param>
    /// <param name="e"></param>
    internal void TimerElapsed(object? sender, ElapsedEventArgs e)
    {
        if (socket.State != WebSocketState.Open)
        {
            Connect();
        }
    }

    /// <summary>
    /// The method <c>Update</c> updates the address and port of the server.
    /// </summary>
    /// <param name="address"></param>
    /// <param name="port"></param>
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

    /// <summary>
    /// The method <c>Connect</c> connects to the server.
    /// </summary>
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

    /// <summary>
    /// The method <c>Disconnect</c> disconnects from the server.
    /// </summary>
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

    /// <summary>
    /// The method <c>ReceiveMessagesAsync</c> receives messages from the server asynchronously.
    /// </summary>
    /// <returns></returns>
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

    /// <summary>
    /// The method <c>OnMessageReceived</c> raises the MessageReceived event.
    /// </summary>
    /// <param name="message"></param>
    protected virtual void OnMessageReceived(string message)
    {
        MessageReceived?.Invoke(this, message);
    }

    /// <summary>
    /// The method <c>OnConnectionChanged</c> raises the ConnectionChanged event.
    /// </summary>
    /// <param name="state"></param>
    protected virtual void OnConnectionChanged(ConnectionState state)
    {
        ConnectionChanged?.Invoke(this, state);
    }
}
