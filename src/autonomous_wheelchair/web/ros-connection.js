// 공용 rosbridge 연결 헬퍼. vendor/roslib.min.js 로드 뒤에 포함할 것.
// 사용법: WheelchairUi.connect(function (ros) { ... });
// 태블릿이 라즈베리파이와 다른 호스트에서 접속할 수 있으므로 ?host=<pi-ip> 로 override 가능,
// 기본은 현재 페이지를 서빙 중인 호스트(location.hostname)를 그대로 사용.
(function (global) {
  function resolveWsUrl() {
    var params = new URLSearchParams(window.location.search);
    var host = params.get('host') || window.location.hostname || 'localhost';
    var port = params.get('port') || '9090';
    return 'ws://' + host + ':' + port;
  }

  function connect(onConnected) {
    var url = resolveWsUrl();
    var ros = new ROSLIB.Ros({ url: url });
    ros.on('connection', function () {
      console.log('rosbridge 연결됨:', url);
      if (onConnected) onConnected(ros);
    });
    ros.on('error', function (err) {
      console.error('rosbridge 연결 오류', err);
    });
    ros.on('close', function () {
      console.warn('rosbridge 연결 끊김, 3초 후 재연결 시도');
      setTimeout(function () { ros.connect(url); }, 3000);
    });
    return ros;
  }

  function publish(ros, name, messageType, data) {
    var topic = new ROSLIB.Topic({ ros: ros, name: name, messageType: messageType });
    topic.publish(new ROSLIB.Message(data));
  }

  global.WheelchairUi = global.WheelchairUi || {};
  global.WheelchairUi.connect = connect;
  global.WheelchairUi.publish = publish;
  global.WheelchairUi.resolveWsUrl = resolveWsUrl;
})(window);
