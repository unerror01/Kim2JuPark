// 목적지 버튼 목록 렌더링 (destination.html, manual.html 공용).
// /waypoints_list 는 ui_bridge 가 transient_local 로 발행하므로 늦게 구독해도 바로 값을 받는다.
(function (global) {
  function renderDestinationList(ros, containerEl, onSelect) {
    var topic = new ROSLIB.Topic({
      ros: ros,
      name: '/waypoints_list',
      messageType: 'std_msgs/String',
    });
    topic.subscribe(function (msg) {
      var waypoints;
      try {
        waypoints = JSON.parse(msg.data);
      } catch (e) {
        console.error('waypoints_list 파싱 실패', e);
        return;
      }
      containerEl.innerHTML = '';
      if (!waypoints || waypoints.length === 0) {
        containerEl.innerHTML = '<p class="empty">등록된 목적지가 없습니다. 관리자 페이지에서 추가해주세요.</p>';
        return;
      }
      waypoints.forEach(function (wp) {
        var btn = document.createElement('button');
        btn.className = 'dest-btn';
        btn.textContent = wp.name;
        btn.addEventListener('click', function () { onSelect(wp.name); });
        containerEl.appendChild(btn);
      });
    });
    return topic;
  }

  global.WheelchairUi = global.WheelchairUi || {};
  global.WheelchairUi.renderDestinationList = renderDestinationList;
})(window);
