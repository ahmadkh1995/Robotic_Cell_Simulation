graph TB
    subgraph "WMS Server Container (Port 5000)"
        WMS[WMS Server<br/>Flask Application]
    end
    
    subgraph "ROS2 Environment"
        SIM[Robotic Cell Simulator<br/>Port 8080]
        BS[Barcode Scanner<br/>ROS2 Node]
        DH[Door Handler<br/>ROS2 Node]
        EH[Emergency Handler<br/>ROS2 Node]
        SL[Stack Light Handler<br/>ROS2 Node]
        DDS[No subscriber only to ROS2 DDS layer]
        
    end
    
    subgraph "GUI Applications"
        GUI1[WMS Robotic Arm HMI<br/>Port 8081]
        GUI2[Robot Cell Control Panel<br/>Port 8082]
    end
    
    subgraph "External Systems"
        CLIENT[External Client<br/>API Requests]
    end

    CLIENT -->|HTTP Requests| WMS
    WMS -->|Pick Requests| SIM
    SIM -->|Pick Confirmations| WMS
    
    SIM -->|Service: 'barcode_scanner/get_barcode'| BS
    SIM -->|Service: 'robotic_cell/door_control'| DH
    DH -->|Topic: 'robotic_cell/door_state'| SIM
    SIM -->|Service: 'robotic_cell/emergency_control'| EH
    EH -->|Topic: 'robotic_cell/emergency_state'| SIM
    SIM -->|Service: 'robotic_cell/get_status'| SL
    SL -->|Topic: 'stack_light/status'| DDS
    BS -->|Topic: 'barcode_scanner/barcode'| DDS

    

    GUI1 -->|Monitor| WMS
    GUI1 -->|Status Polling| SIM
    GUI2 -->|Control Commands| SIM

