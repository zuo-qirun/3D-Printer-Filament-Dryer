using System;
using System.Net.Http;
using System.Net.Http.Json;
using System.Text;
using System.Text.Json;
using System.Threading.Tasks;
using System.Windows;
using System.Windows.Media;
using System.Windows.Threading;
using MQTTnet;
using MQTTnet.Protocol;

namespace FilamentDryerMonitor;

public partial class MainWindow : Window
{
    private readonly HttpClient _httpClient = new()
    {
        Timeout = TimeSpan.FromSeconds(4),
    };

    private readonly DispatcherTimer _pollTimer;
    private string _baseUrl = string.Empty;
    private IMqttClient? _mqttClient;
    private string _statusTopic = "dryer/esp32/status";
    private string _controlTopic = "dryer/esp32/control";

    public MainWindow()
    {
        InitializeComponent();

        _pollTimer = new DispatcherTimer
        {
            Interval = TimeSpan.FromSeconds(1),
        };
        _pollTimer.Tick += async (_, _) => await PollStatusAsync();
    }

    private async void ConnectButton_Click(object sender, RoutedEventArgs e)
    {
        string raw = AddressTextBox.Text.Trim();
        if (string.IsNullOrWhiteSpace(raw))
        {
            MessageBox.Show("请输入 ESP32 地址，例如 192.168.4.1", "地址为空", MessageBoxButton.OK, MessageBoxImage.Warning);
            return;
        }

        _baseUrl = raw.StartsWith("http://", StringComparison.OrdinalIgnoreCase)
            || raw.StartsWith("https://", StringComparison.OrdinalIgnoreCase)
            ? raw
            : $"http://{raw}";

        _baseUrl = _baseUrl.TrimEnd('/');
        SetConnectingState(true, "LAN: 连接中...");

        bool ok = await PollStatusAsync();
        if (!ok)
        {
            SetConnectingState(false, "LAN: 连接失败");
            return;
        }

        _pollTimer.Start();
        ConnectButton.IsEnabled = false;
        DisconnectButton.IsEnabled = true;
        AddressTextBox.IsEnabled = false;
        SetConnectedVisuals(true, "LAN: 已连接");
    }

    private void DisconnectButton_Click(object sender, RoutedEventArgs e)
    {
        _pollTimer.Stop();
        _baseUrl = string.Empty;
        ConnectButton.IsEnabled = true;
        DisconnectButton.IsEnabled = false;
        AddressTextBox.IsEnabled = true;
        SetConnectedVisuals(false, "LAN: 未连接");
    }

    private async void MqttConnectButton_Click(object sender, RoutedEventArgs e)
    {
        string host = MqttHostTextBox.Text.Trim();
        string portText = MqttPortTextBox.Text.Trim();
        string key = MqttKeyTextBox.Text.Trim();
        _statusTopic = MqttStatusTopicTextBox.Text.Trim();
        _controlTopic = MqttControlTopicTextBox.Text.Trim();

        if (string.IsNullOrWhiteSpace(host) || string.IsNullOrWhiteSpace(portText) || string.IsNullOrWhiteSpace(key))
        {
            MessageBox.Show("MQTT 主机、端口和私钥不能为空。", "MQTT 参数错误", MessageBoxButton.OK, MessageBoxImage.Warning);
            return;
        }

        if (string.IsNullOrWhiteSpace(_statusTopic) || string.IsNullOrWhiteSpace(_controlTopic))
        {
            MessageBox.Show("MQTT 主题不能为空。", "MQTT 参数错误", MessageBoxButton.OK, MessageBoxImage.Warning);
            return;
        }

        if (!int.TryParse(portText, out int port) || port <= 0 || port > 65535)
        {
            MessageBox.Show("MQTT 端口无效。", "MQTT 参数错误", MessageBoxButton.OK, MessageBoxImage.Warning);
            return;
        }

        try
        {
            _mqttClient ??= new MqttClientFactory().CreateMqttClient();
            _mqttClient.ApplicationMessageReceivedAsync -= OnMqttMessageReceivedAsync;
            _mqttClient.ConnectedAsync -= OnMqttConnectedAsync;
            _mqttClient.DisconnectedAsync -= OnMqttDisconnectedAsync;
            _mqttClient.ApplicationMessageReceivedAsync += OnMqttMessageReceivedAsync;
            _mqttClient.ConnectedAsync += OnMqttConnectedAsync;
            _mqttClient.DisconnectedAsync += OnMqttDisconnectedAsync;

            var options = new MqttClientOptionsBuilder()
                .WithClientId($"dryer-desktop-{Guid.NewGuid():N}")
                .WithTcpServer(host, port)
                .WithCredentials(key, string.Empty)
                .WithCleanSession()
                .Build();

            SetMqttVisuals(false, "MQTT: 连接中...");
            await _mqttClient.ConnectAsync(options);

            MqttConnectButton.IsEnabled = false;
            MqttDisconnectButton.IsEnabled = true;
            SetMqttVisuals(true, $"MQTT: 已连接 ({host}:{port})");
        }
        catch (Exception ex)
        {
            SetMqttVisuals(false, "MQTT: 连接失败");
            MessageBox.Show($"MQTT 连接失败: {ex.Message}", "MQTT 错误", MessageBoxButton.OK, MessageBoxImage.Error);
        }
    }

    private async void MqttDisconnectButton_Click(object sender, RoutedEventArgs e)
    {
        if (_mqttClient is not null && _mqttClient.IsConnected)
        {
            try
            {
                await _mqttClient.DisconnectAsync();
            }
            catch
            {
                // ignore
            }
        }

        MqttConnectButton.IsEnabled = true;
        MqttDisconnectButton.IsEnabled = false;
        SetMqttVisuals(false, "MQTT: 未连接");
    }

    private Task OnMqttConnectedAsync(MqttClientConnectedEventArgs arg)
    {
        _ = Dispatcher.InvokeAsync(async () =>
        {
            if (_mqttClient is null || !_mqttClient.IsConnected)
            {
                return;
            }

            await _mqttClient.SubscribeAsync(new MqttTopicFilterBuilder()
                .WithTopic(_statusTopic)
                .WithQualityOfServiceLevel(MqttQualityOfServiceLevel.AtMostOnce)
                .Build());

            DataSourceText.Text = $"数据来源: MQTT 订阅 {_statusTopic}";
        });

        return Task.CompletedTask;
    }

    private Task OnMqttDisconnectedAsync(MqttClientDisconnectedEventArgs arg)
    {
        _ = Dispatcher.InvokeAsync(() =>
        {
            MqttConnectButton.IsEnabled = true;
            MqttDisconnectButton.IsEnabled = false;
            SetMqttVisuals(false, "MQTT: 已断开");
        });

        return Task.CompletedTask;
    }

    private Task OnMqttMessageReceivedAsync(MqttApplicationMessageReceivedEventArgs arg)
    {
        if (!string.Equals(arg.ApplicationMessage.Topic, _statusTopic, StringComparison.Ordinal))
        {
            return Task.CompletedTask;
        }

        string payload = arg.ApplicationMessage.ConvertPayloadToString();

        _ = Dispatcher.InvokeAsync(() =>
        {
            try
            {
                StatusSnapshot? status = JsonSerializer.Deserialize<StatusSnapshot>(payload);
                if (status is null)
                {
                    return;
                }

                UpdateUi(status, payload, "MQTT");
            }
            catch
            {
                // ignore malformed payload
            }
        });

        return Task.CompletedTask;
    }

    private async void StartButton_Click(object sender, RoutedEventArgs e) => await SendControlCommandAsync("start");

    private async void StopButton_Click(object sender, RoutedEventArgs e) => await SendControlCommandAsync("stop");

    private async void FaultResetButton_Click(object sender, RoutedEventArgs e) => await SendControlCommandAsync("faultreset");

    private async void AutoTuneButton_Click(object sender, RoutedEventArgs e) => await SendControlCommandAsync("autotune");

    private async void ApplyIdleTempButton_Click(object sender, RoutedEventArgs e)
    {
        string raw = IdleTempSetTextBox.Text.Trim();
        if (!double.TryParse(raw, out double idleTemp))
        {
            MessageBox.Show("闲时温度请输入数字（0 或 35~120）。", "输入错误", MessageBoxButton.OK, MessageBoxImage.Warning);
            return;
        }

        if (!(idleTemp == 0 || (idleTemp >= 35 && idleTemp <= 120)))
        {
            MessageBox.Show("闲时温度范围应为 0（关闭）或 35~120。", "输入错误", MessageBoxButton.OK, MessageBoxImage.Warning);
            return;
        }

        await SendControlPayloadAsync(new { idle_temp_c = idleTemp }, $"闲时温度已发送: {idleTemp:F1}C");
    }

    private async Task SendControlCommandAsync(string command)
    {
        await SendControlPayloadAsync(new { cmd = command }, $"控制命令已发送: {command}");
    }

    private async Task SendControlPayloadAsync(object payload, string sourceHint)
    {
        try
        {
            if (_mqttClient is not null && _mqttClient.IsConnected)
            {
                string message = JsonSerializer.Serialize(payload);
                var mqttMessage = new MqttApplicationMessageBuilder()
                    .WithTopic(_controlTopic)
                    .WithPayload(message)
                    .WithQualityOfServiceLevel(MqttQualityOfServiceLevel.AtMostOnce)
                    .Build();

                await _mqttClient.PublishAsync(mqttMessage);
                DataSourceText.Text = $"数据来源: {sourceHint} (MQTT {_controlTopic})";
                return;
            }

            if (string.IsNullOrEmpty(_baseUrl))
            {
                MessageBox.Show("请先连接 LAN 或 MQTT。", "未连接", MessageBoxButton.OK, MessageBoxImage.Information);
                return;
            }

            using HttpResponseMessage resp = await _httpClient.PostAsJsonAsync($"{_baseUrl}/api/control", payload);
            resp.EnsureSuccessStatusCode();
            await PollStatusAsync();
        }
        catch (Exception ex)
        {
            MessageBox.Show($"发送控制命令失败: {ex.Message}", "请求失败", MessageBoxButton.OK, MessageBoxImage.Error);
        }
    }

    private async Task<bool> PollStatusAsync()
    {
        if (string.IsNullOrEmpty(_baseUrl))
        {
            return false;
        }

        try
        {
            using HttpResponseMessage resp = await _httpClient.GetAsync($"{_baseUrl}/api/status");
            resp.EnsureSuccessStatusCode();
            string json = await resp.Content.ReadAsStringAsync();

            StatusSnapshot? status = JsonSerializer.Deserialize<StatusSnapshot>(json);
            if (status is null)
            {
                throw new InvalidOperationException("状态 JSON 解析失败。");
            }

            UpdateUi(status, json, "LAN HTTP");
            return true;
        }
        catch
        {
            if (_pollTimer.IsEnabled)
            {
                _pollTimer.Stop();
                ConnectButton.IsEnabled = true;
                DisconnectButton.IsEnabled = false;
                AddressTextBox.IsEnabled = true;
            }

            SetConnectedVisuals(false, "LAN: 连接中断");
            return false;
        }
    }

    private void UpdateUi(StatusSnapshot status, string rawJson, string source)
    {
        TempText.Text = $"{status.temp_c:F1} C";
        HumiText.Text = $"{status.humi_pct:F1} %";
        TargetText.Text = $"{status.target_c:F1} C";
        IdleTempText.Text = status.idle_temp_c < 35 ? "OFF" : $"{status.idle_temp_c:F1} C";
        IdleTempSetTextBox.Text = status.idle_temp_c < 35 ? "0" : status.idle_temp_c.ToString("F1");
        RemainingText.Text = status.remaining_hms;

        string runState = status.active ? "运行中" : "已停止";
        RunStateText.Text = $"运行状态: {runState} | 预设: {status.preset} | 加热: {status.heater_pct}% | 风扇PWM: {status.fan_pct}% | 风扇转速: {status.fan_rpm} RPM";

        string wifiState = status.wifi_connected ? "已连接路由器" : (status.ap_mode ? "AP 配网模式" : "离线");
        NetworkText.Text = $"网络: {wifiState} | IP: {status.ip}";

        FaultText.Text = $"故障标志: {status.fault} | PID 自整定: {(status.pid_autotune ? "进行中" : status.pid_autotune_msg)} ({status.pid_autotune_progress}%)";
        DataSourceText.Text = $"数据来源: {source}";
        RawJsonTextBox.Text = PrettyPrintJson(rawJson);
    }

    private static string PrettyPrintJson(string raw)
    {
        using JsonDocument doc = JsonDocument.Parse(raw);
        return JsonSerializer.Serialize(doc, new JsonSerializerOptions { WriteIndented = true });
    }

    private void SetConnectingState(bool connecting, string text)
    {
        ConnectionStateText.Text = text;
        ConnectionStateText.Foreground = connecting ? new SolidColorBrush(Color.FromRgb(245, 158, 11)) : new SolidColorBrush(Color.FromRgb(248, 113, 113));
    }

    private void SetConnectedVisuals(bool connected, string text)
    {
        ConnectionStateText.Text = text;
        ConnectionStateText.Foreground = connected ? new SolidColorBrush(Color.FromRgb(74, 222, 128)) : new SolidColorBrush(Color.FromRgb(245, 158, 11));
    }

    private void SetMqttVisuals(bool connected, string text)
    {
        MqttStateText.Text = text;
        MqttStateText.Foreground = connected ? new SolidColorBrush(Color.FromRgb(74, 222, 128)) : new SolidColorBrush(Color.FromRgb(245, 158, 11));
    }

    protected override async void OnClosed(EventArgs e)
    {
        _pollTimer.Stop();

        if (_mqttClient is not null)
        {
            try
            {
                if (_mqttClient.IsConnected)
                {
                    await _mqttClient.DisconnectAsync();
                }
            }
            catch
            {
                // ignore
            }

            _mqttClient.Dispose();
        }

        _httpClient.Dispose();
        base.OnClosed(e);
    }

    private sealed record StatusSnapshot(
        double temp_c,
        double humi_pct,
        double target_c,
        string remaining_hms,
        string preset,
        bool active,
        int heater_pct,
        int fan_pct,
        int fault,
        bool pid_autotune,
        string pid_autotune_msg,
        int pid_autotune_progress,
        bool wifi_connected,
        bool ap_mode,
        string ip,
        double idle_temp_c = 0,
        int fan_rpm = 0);
}
